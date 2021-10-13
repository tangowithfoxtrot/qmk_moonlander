/*
  Set any config.h overrides for your specific keymap here.
  See config.h options at https://docs.qmk.fm/#/config_options?id=the-configh-file
*/
#define ORYX_CONFIGURATOR
#define USB_SUSPEND_WAKEUP_DELAY 0
#undef MOUSEKEY_WHEEL_MAX_SPEED
#define MOUSEKEY_WHEEL_MAX_SPEED 5

#undef MOUSEKEY_WHEEL_TIME_TO_MAX
#define MOUSEKEY_WHEEL_TIME_TO_MAX 55

#define CAPS_LOCK_STATUS
#define FIRMWARE_VERSION u8"mqNKd/bjQrA"
#define RGB_MATRIX_STARTUP_SPD 60

#ifdef AUDIO_ENABLE
#    define STARTUP_SONG SONG(MARIO_MUSHROOM)
#    define AUDIO_VOICES
#    define AUDIO_ENABLE_TONE_MULTIPLEXING
#    define AUDIO_TONE_MULTIPLEXING_RATE_DEFAULT 10
#endif

