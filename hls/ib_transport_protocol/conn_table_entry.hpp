#pragma once

#include "../axi_utils.hpp"

struct connTableEntry
{
	//ap_uint<24> local_qpn;
	ap_uint<24> remote_qpn;
	ap_uint<128> remote_ip_address; //TODO make variable
	ap_uint<16> remote_udp_port; //TODO what is this used for
};
