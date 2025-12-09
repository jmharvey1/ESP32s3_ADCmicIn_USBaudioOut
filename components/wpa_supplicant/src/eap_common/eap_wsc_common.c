/*
 * EAP-WSC common routines for Wi-Fi Protected Setup
 * Copyright (c) 2007, Jouni Malinen <j@w1.fi>
 *
 * This software may be distributed under the terms of the BSD license.
 * See README for more details.
 */

//#include "includes.h"
#include "../utils/includes.h" //JMH ADDED
//#include "common.h"
#include "../../include/utils/common.h"//JMH ADDED
//#include "eap_peer/eap_defs.h"
#include "../src/eap_peer/eap_defs.h"//JMH ADDED
//#include "wps/wps.h"
#include "../src/wps/wps.h"//JMH ADDED
#include "eap_wsc_common.h"
#include "../src/eap_peer/eap_common.h"//JMH ADDED
struct wpabuf * eap_wsc_build_frag_ack(u8 id, u8 code)
{
	struct wpabuf *msg;

	msg = eap_msg_alloc(EAP_VENDOR_WFA, EAP_VENDOR_TYPE_WSC, 2, code, id);
	if (msg == NULL) {
		wpa_printf(MSG_ERROR, "EAP-WSC: Failed to allocate memory for "
			   "FRAG_ACK");
		return NULL;
	}

	wpa_printf(MSG_DEBUG, "EAP-WSC: Send WSC/FRAG_ACK");
	wpabuf_put_u8(msg, WSC_FRAG_ACK); /* Op-Code */
	wpabuf_put_u8(msg, 0); /* Flags */

	return msg;
}
