```
esp-idf/main/libmain.a(main.c.obj): in function `app_main':
scanner/main/main.c:86:(.text.app_main+0xbc): undefined reference to `esp_ble_gap_set_scan_params'
collect2: error: ld returned 1 exit status
```

`CONFIG_BT_BLE_42_FEATURES_SUPPORTED=y` was missing in sdkconfig