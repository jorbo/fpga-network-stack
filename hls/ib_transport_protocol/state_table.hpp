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

#include "../axi_utils.hpp"
#include "state_table_structs.hpp"
#include <rocev2_config.hpp> //defines MAX_QPS
#include "ib_types.hpp"
template <int SIZE>
void state_table(hls::stream<rxStateReq> rxIbh2stateTable_upd_req[SIZE],
				 hls::stream<txStateReq> txIbh2stateTable_upd_req[SIZE],
				 hls::stream<ifStateReq> qpi2stateTable_upd_req[SIZE],
				 hls::stream<rxStateRsp> stateTable2rxIbh_rsp[SIZE],
				 hls::stream<stateTableEntry> stateTable2txIbh_rsp[SIZE],
				 hls::stream<stateTableEntry> stateTable2qpi_rsp[SIZE])
{
#pragma HLS ARRAY_PARTITION variable=rxIbh2stateTable_upd_req dim=1 complete    
#pragma HLS ARRAY_PARTITION variable=txIbh2stateTable_upd_req dim=1 complete    
#pragma HLS ARRAY_PARTITION variable=qpi2stateTable_upd_req dim=1 complete   
#pragma HLS ARRAY_PARTITION variable=stateTable2rxIbh_rsp dim=1 complete    
#pragma HLS ARRAY_PARTITION variable=stateTable2txIbh_rsp dim=1 complete    
#pragma HLS ARRAY_PARTITION variable=stateTable2qpi_rsp dim=1 complete   

#pragma HLS PIPELINE II = 1
#pragma HLS INLINE off


	static stateTableEntry state_table[MAX_QPS];
#pragma HLS RESOURCE variable = state_table core = RAM_2P_BRAM

	for (int i = 0; i < SIZE; i++)
	{
		#pragma HLS unroll

		rxStateReq rxRequest;
		txStateReq txRequest;
		ifStateReq ifRequest;

		if (!rxIbh2stateTable_upd_req[i].empty())
		{
			rxIbh2stateTable_upd_req[i].read(rxRequest);
			if (rxRequest.write)
			{
				if (rxRequest.isResponse)
				{
					state_table[rxRequest.qpn].req_old_unack = rxRequest.epsn;
				}
				else
				{
					state_table[rxRequest.qpn].resp_epsn = rxRequest.epsn;
					state_table[rxRequest.qpn].retryCounter = rxRequest.retryCounter;
					// state_table[rxRequest.qpn].sendNAK = rxRequest.epsn;
				}
			}
			else
			{
				stateTableEntry entry = state_table[rxRequest.qpn(15, 0)];
				if (rxRequest.isResponse)
				{
					stateTable2rxIbh_rsp[i].write(rxStateRsp(entry.req_old_unack, entry.req_old_valid, entry.req_next_psn - 1));
				}
				else
				{
					stateTable2rxIbh_rsp[i].write(rxStateRsp(entry.resp_epsn, entry.resp_old_outstanding, entry.resp_epsn, entry.retryCounter));
				}
			}
		}
		else if (!txIbh2stateTable_upd_req[i].empty())
		{
			txIbh2stateTable_upd_req[i].read(txRequest);
			if (txRequest.write)
			{
				state_table[txRequest.qpn].req_next_psn = txRequest.psn;
			}
			else
			{
				stateTable2txIbh_rsp[i].write(state_table[txRequest.qpn]);
			}
		}
		else if (!qpi2stateTable_upd_req[i].empty())
		{
			qpi2stateTable_upd_req[i].read(ifRequest);
			if (ifRequest.write)
			{
				std::cout << "SETUP new connection, PSN: " << ifRequest.remote_psn << std::endl;
				// state_table[ifRequest.qpn].state = ifRequest.newState;
				// state_table[ifRequest.qpn].prevOpCode = RC_RDMA_WRITE_LAST;
				state_table[ifRequest.qpn].resp_epsn = ifRequest.local_psn;
				state_table[ifRequest.qpn].resp_old_outstanding = ifRequest.local_psn;
				state_table[ifRequest.qpn].req_next_psn = ifRequest.remote_psn;
				state_table[ifRequest.qpn].req_old_unack = ifRequest.remote_psn;
				state_table[ifRequest.qpn].req_old_valid = ifRequest.remote_psn;
				state_table[ifRequest.qpn].retryCounter = 0xF;
				// state_table[ifRequest.qpn].sendNAK = false;

				// state_table[ifRequest.qpn].r_key = ifRequest.r_key;
				// state_table[ifRequest.qpn].virtual_address = ifRequest.virtual_address;
			}
			else
			{
				stateTable2qpi_rsp[i].write(state_table[ifRequest.qpn(15, 0)]);
			}
		}
	}
}
