#include "driver_api.h"
#include "utils.h"

#define GCA_DEBUG 0

/* Nintendo GameCube Controller Adapter (WUP-028) and clones in native mode
 * (e.g. Mayflash set to "Wii U").
 *
 * Protocol: a single interrupt OUT byte 0x13 starts the polling, after which
 * the adapter sends 37-byte interrupt IN reports: 0x21 followed by 9 bytes
 * per port:
 *   [status, buttons1, buttons2, stick_x, stick_y, cstick_x, cstick_y, l, r]
 * status bit 4 (0x10) = wired controller, bit 5 (0x20) = wireless.
 * Rumble: interrupt OUT [0x11, p1, p2, p3, p4] with 1/0 per port. */

#define GCA_NUM_PORTS 4
#define GCA_REPORT_SIZE 37
#define GCA_EP_IN (EGC_USB_ENDPOINT_IN | 1)
#define GCA_EP_OUT (EGC_USB_ENDPOINT_OUT | 2)

enum gca_buttons_e {
    /* buttons1 */
    GCA_BUTTON_A,
    GCA_BUTTON_B,
    GCA_BUTTON_X,
    GCA_BUTTON_Y,
    GCA_BUTTON_DPAD_LEFT,
    GCA_BUTTON_DPAD_RIGHT,
    GCA_BUTTON_DPAD_DOWN,
    GCA_BUTTON_DPAD_UP,
    /* buttons2 */
    GCA_BUTTON_START,
    GCA_BUTTON_Z,
    GCA_BUTTON_R,
    GCA_BUTTON_L,
    GCA_BUTTON_COUNT
};

struct gca_private_data_t {
    u8 active_port;
    bool rumble_on;
};
static_assert(sizeof(struct gca_private_data_t) <= EGC_INPUT_DEVICE_PRIVATE_DATA_SIZE);

/* GameCube A is the primary/confirm button: map it to SOUTH, which fakemote
 * turns into Wiimote A / Classic Controller B. Z+Start together are
 * synthesized into LEFT_STICK, which fakemote maps to the HOME button. */
static const egc_gamepad_button_e s_button_map[GCA_BUTTON_COUNT] = {
    [GCA_BUTTON_A] = EGC_GAMEPAD_BUTTON_SOUTH,
    [GCA_BUTTON_B] = EGC_GAMEPAD_BUTTON_EAST,
    [GCA_BUTTON_X] = EGC_GAMEPAD_BUTTON_WEST,
    [GCA_BUTTON_Y] = EGC_GAMEPAD_BUTTON_NORTH,
    [GCA_BUTTON_DPAD_LEFT] = EGC_GAMEPAD_BUTTON_DPAD_LEFT,
    [GCA_BUTTON_DPAD_RIGHT] = EGC_GAMEPAD_BUTTON_DPAD_RIGHT,
    [GCA_BUTTON_DPAD_DOWN] = EGC_GAMEPAD_BUTTON_DPAD_DOWN,
    [GCA_BUTTON_DPAD_UP] = EGC_GAMEPAD_BUTTON_DPAD_UP,
    [GCA_BUTTON_START] = EGC_GAMEPAD_BUTTON_START,
    [GCA_BUTTON_Z] = EGC_GAMEPAD_BUTTON_BACK,
    [GCA_BUTTON_R] = EGC_GAMEPAD_BUTTON_RIGHT_SHOULDER,
    [GCA_BUTTON_L] = EGC_GAMEPAD_BUTTON_LEFT_SHOULDER,
};

static const egc_device_description_t s_device_description = {
    .vendor_id = 0x057e,
    .product_id = 0x0337,
    /* clang-format off */
    .available_buttons =
        BIT(EGC_GAMEPAD_BUTTON_SOUTH) |
        BIT(EGC_GAMEPAD_BUTTON_EAST) |
        BIT(EGC_GAMEPAD_BUTTON_WEST) |
        BIT(EGC_GAMEPAD_BUTTON_NORTH) |
        BIT(EGC_GAMEPAD_BUTTON_DPAD_UP) |
        BIT(EGC_GAMEPAD_BUTTON_DPAD_DOWN) |
        BIT(EGC_GAMEPAD_BUTTON_DPAD_LEFT) |
        BIT(EGC_GAMEPAD_BUTTON_DPAD_RIGHT) |
        BIT(EGC_GAMEPAD_BUTTON_START) |
        BIT(EGC_GAMEPAD_BUTTON_BACK) |
        BIT(EGC_GAMEPAD_BUTTON_LEFT_SHOULDER) |
        BIT(EGC_GAMEPAD_BUTTON_RIGHT_SHOULDER) |
        BIT(EGC_GAMEPAD_BUTTON_LEFT_STICK),
    .available_axes =
        BIT(EGC_GAMEPAD_AXIS_LEFTX) |
        BIT(EGC_GAMEPAD_AXIS_LEFTY) |
        BIT(EGC_GAMEPAD_AXIS_RIGHTX) |
        BIT(EGC_GAMEPAD_AXIS_RIGHTY) |
        BIT(EGC_GAMEPAD_AXIS_LEFT_TRIGGER) |
        BIT(EGC_GAMEPAD_AXIS_RIGHT_TRIGGER),
    /* clang-format on */
    .type = EGC_DEVICE_TYPE_GAMEPAD,
    .num_touch_points = 0,
    .num_leds = 0,
    .num_accelerometers = 0,
    .has_rumble = true,
};

static int gca_request_data(egc_input_device_t *device);

static void intr_transfer_cb(egc_usb_transfer_t *transfer)
{
    egc_input_device_t *device = transfer->device;
    struct gca_private_data_t *priv = (void *)device->private_data;
    const u8 *report = transfer->data;
    egc_input_state_t state = { 0 };

#if GCA_DEBUG
    LOG_INFO("gca: %02x | %02x %02x %02x %02x %02x %02x %02x %02x %02x | st=%d len=%d\n",
             report[0], report[1], report[2], report[3], report[4], report[5], report[6], report[7],
             report[8], report[9], transfer->status, transfer->length);
#endif
    if (transfer->status == EGC_USB_TRANSFER_STATUS_COMPLETED && report[0] == 0x21) {
        /* use the first port with a controller plugged in */
        const u8 *pad = NULL;
        for (int port = 0; port < GCA_NUM_PORTS; port++) {
            const u8 *data = &report[1 + 9 * port];
            if (data[0] & 0x30) {
                pad = data;
                priv->active_port = port;
                break;
            }
        }

        if (pad) {
            u16 buttons = pad[1] | (pad[2] << 8);

            /* Z+Start = HOME (LEFT_STICK in the fakemote mapping) */
            bool home = (buttons & (BIT(GCA_BUTTON_Z) | BIT(GCA_BUTTON_START))) ==
                        (BIT(GCA_BUTTON_Z) | BIT(GCA_BUTTON_START));
            if (home)
                buttons &= ~(BIT(GCA_BUTTON_Z) | BIT(GCA_BUTTON_START));

            state.gamepad.buttons =
                egc_device_driver_map_buttons(buttons, GCA_BUTTON_COUNT, s_button_map);
            if (home)
                state.gamepad.buttons |= BIT(EGC_GAMEPAD_BUTTON_LEFT_STICK);

            /* GameCube sticks report up/right as high values, which matches
             * the egc convention, so no inversion is needed. */
            state.gamepad.axes[EGC_GAMEPAD_AXIS_LEFTX] = egc_u8_to_s16(pad[3]);
            state.gamepad.axes[EGC_GAMEPAD_AXIS_LEFTY] = egc_u8_to_s16(pad[4]);
            state.gamepad.axes[EGC_GAMEPAD_AXIS_RIGHTX] = egc_u8_to_s16(pad[5]);
            state.gamepad.axes[EGC_GAMEPAD_AXIS_RIGHTY] = egc_u8_to_s16(pad[6]);
            state.gamepad.axes[EGC_GAMEPAD_AXIS_LEFT_TRIGGER] = egc_u8_to_s16(pad[7]);
            state.gamepad.axes[EGC_GAMEPAD_AXIS_RIGHT_TRIGGER] = egc_u8_to_s16(pad[8]);
        }

        egc_device_driver_report_input(device, &state);
    }

    gca_request_data(device);
}

static int gca_request_data(egc_input_device_t *device)
{
    /* Request exactly the report size: bigger requests complete with an
     * overflow/error status on some USB stacks. */
    static u8 report_buf[GCA_REPORT_SIZE];
    const egc_usb_transfer_t *transfer = egc_device_driver_issue_intr_transfer_async(
        device, GCA_EP_IN, report_buf, GCA_REPORT_SIZE, intr_transfer_cb);
    return transfer != NULL ? 0 : -1;
}

static void start_polling_cb(egc_usb_transfer_t *transfer)
{
#if GCA_DEBUG
    LOG_INFO("gca: poll cmd sent, status=%d\n", transfer->status);
#endif
    gca_request_data(transfer->device);
}

static bool gca_driver_ops_probe(u16 vid, u16 pid)
{
    static const egc_device_id_t compatible[] = {
        { 0x057e, 0x0337 }, /* Nintendo | GameCube Controller Adapter */
    };

    bool ok = egc_device_driver_is_compatible(vid, pid, compatible, ARRAY_SIZE(compatible));
    LOG_INFO("gca: probe %04x:%04x -> %d\n", vid, pid, ok);
    return ok;
}

static int gca_driver_ops_init(egc_input_device_t *device, u16 vid, u16 pid)
{
    struct gca_private_data_t *priv = (void *)device->private_data;

    LOG_INFO("gca: init called\n");
    priv->active_port = 0;
    priv->rumble_on = false;
    device->desc = &s_device_description;

    /* Let the device settle before starting the polling */
    egc_device_driver_set_timer(device, 1000 * 500, 0);
    return 0;
}

static bool gca_driver_ops_timer(egc_input_device_t *device)
{
    static u8 poll_cmd[1] = { 0x13 };
#if GCA_DEBUG
    LOG_INFO("gca: timer fired, sending poll cmd\n");
#endif
    egc_device_driver_issue_intr_transfer_async(device, GCA_EP_OUT, poll_cmd, sizeof(poll_cmd),
                                                start_polling_cb);
    /* Return false to destroy the timer */
    return false;
}

static int gca_driver_ops_set_rumble(egc_input_device_t *device, bool rumble_on)
{
    struct gca_private_data_t *priv = (void *)device->private_data;
    u8 rumble_cmd[5] = { 0x11, 0, 0, 0, 0 };

    priv->rumble_on = rumble_on;
    rumble_cmd[1 + priv->active_port] = rumble_on ? 1 : 0;
    egc_device_driver_issue_intr_transfer_async(device, GCA_EP_OUT, rumble_cmd, sizeof(rumble_cmd),
                                                NULL);
    return 0;
}

const egc_device_driver_t gca_usb_device_driver = {
    .probe = gca_driver_ops_probe,
    .init = gca_driver_ops_init,
    .timer = gca_driver_ops_timer,
    .set_rumble = gca_driver_ops_set_rumble,
};
