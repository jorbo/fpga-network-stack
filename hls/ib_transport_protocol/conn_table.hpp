/************************************************
Copyright (c) 2019, Systems Group, ETH Zurich.
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation
and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors
may be used to endorse or promote products derived from this software
without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
************************************************/
#pragma once
#include "ib_transport_protocol.hpp"
#include "conn_table_entry.hpp"
#include <rocev2_config.hpp> //defines MAX_QPS
template<int SIZE>
void conn_table(	hls::stream<ap_uint<16> >	tx_ibhconnTable_req[SIZE],
						hls::stream<ifConnReq>&		qpi2connTable_req,
						hls::stream<connTableEntry>	tx_connTable2ibh_rsp[SIZE])
{
#pragma HLS ARRAY_PARTITION variable=tx_ibhconnTable_req dim=1 complete    
#pragma HLS ARRAY_PARTITION variable=tx_connTable2ibh_rsp dim=1 complete  
#pragma HLS STREAM depth = 2 variable = tx_connTable2ibh_rsp

#pragma HLS PIPELINE II=1
#pragma HLS INLINE off

	for(int i = 0; i < SIZE; i++){
		#pragma HLS unroll
		static connTableEntry conn_table[MAX_QPS];
		//#pragma HLS RESOURCE variable=conn_table core=RAM_2P_BRAM

		ap_uint<16> txRequest;
		ifConnReq ifRequest;

		if (!tx_ibhconnTable_req[i].empty())
		{
			tx_ibhconnTable_req[i].read(txRequest);
			//std::cout << "Requested conn data for: " << txRequest << std::endl;
			tx_connTable2ibh_rsp[i].write(conn_table[txRequest]);
		}
		else if (!qpi2connTable_req.empty())
		{
			qpi2connTable_req.read(ifRequest);
			//std::cout << "Storing conn data for: " << ifRequest.qpn << std::endl;
			conn_table[ifRequest.qpn].remote_qpn = ifRequest.remote_qpn;
			conn_table[ifRequest.qpn].remote_ip_address = ifRequest.remote_ip_address;
			conn_table[ifRequest.qpn].remote_udp_port = ifRequest.remote_udp_port;
		}
	}
}
