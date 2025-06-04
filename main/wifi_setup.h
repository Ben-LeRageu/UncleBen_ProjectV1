// wifi_setup.h
#ifndef WIFI_SETUP_H
#define WIFI_SETUP_H

void wifi_init_ap_mode(void);
void wifi_init_sta_mode(const char *ssid, const char *password);

#endif // WIFI_SETUP_H