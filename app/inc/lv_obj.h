#pragma once

void create_pages(lv_obj_t *parent);

extern volatile bool dht11_request;
void dht11_update_ui();
