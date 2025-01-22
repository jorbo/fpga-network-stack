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

struct ifMsnReq
{
	ap_uint<16> qpn;
	ap_uint<32> r_key;
	ifMsnReq() {}
	ifMsnReq(ap_uint<16> qpn, ap_uint<32> r_key)
		:qpn(qpn), r_key(r_key) {}
};

struct rxMsnReq
{
	ap_uint<16> qpn;
	ap_uint<24> msn;
	ap_uint<64> vaddr;
	ap_uint<32> dma_length;
	bool		write;
	rxMsnReq()
		:write(false) {}
	rxMsnReq(ap_uint<16> qpn)
		:qpn(qpn), write(false) {}
	rxMsnReq(ap_uint<16> qpn, ap_uint<24> msn)
			:qpn(qpn), msn(msn), vaddr(0), dma_length(0), write(true) {}
	rxMsnReq(ap_uint<16> qpn, ap_uint<24> msn, ap_uint<64> vaddr, ap_uint<32> len)
		:qpn(qpn), msn(msn), vaddr(vaddr), dma_length(len), write(true) {}
};

struct txMsnRsp
{
	ap_uint<24>	msn;
	ap_uint<32> r_key;
	txMsnRsp() {}
	txMsnRsp(ap_uint<24> msn, ap_uint<32> r_key)
		:msn(msn), r_key(r_key) {}
};

/*
 * dma: page 167
 * msn: page 169
 */
struct dmaState
{
	ap_uint<24> msn;
	ap_uint<64> vaddr;
	ap_uint<32> dma_length;
	ap_uint<32> r_key;
};


template <int SIZE>
void msn_table(hls::stream<rxMsnReq> rxExh2msnTable_upd_req[SIZE],
			   hls::stream<ap_uint<16> > txExh2msnTable_req[SIZE],
			   hls::stream<ifMsnReq> if2msnTable_init[SIZE],
			   hls::stream<dmaState> msnTable2rxExh_rsp[SIZE],
			   hls::stream<txMsnRsp> msnTable2txExh_rsp[SIZE])
{
#pragma HLS ARRAY_PARTITION variable=rxExh2msnTable_upd_req dim=1 complete    
#pragma HLS ARRAY_PARTITION variable=txExh2msnTable_req dim=1 complete    
#pragma HLS ARRAY_PARTITION variable=if2msnTable_init dim=1 complete    
#pragma HLS ARRAY_PARTITION variable=msnTable2rxExh_rsp dim=1 complete    
#pragma HLS ARRAY_PARTITION variable=msnTable2txExh_rsp dim=1 complete    

#pragma HLS PIPELINE II = 1
#pragma HLS INLINE off

	static dmaState msn_table[MAX_QPS];
#pragma HLS RESOURCE variable = msn_table core = RAM_2P_BRAM
	for (int i = 0; i < SIZE; i++)
	{
		#pragma HLS unroll
		rxMsnReq rxRequest;
		ifMsnReq ifRequest;
		ap_uint<16> qpn;

		// TODO init channel

		if (!rxExh2msnTable_upd_req[i].empty())
		{
			rxExh2msnTable_upd_req[i].read(rxRequest);
			if (rxRequest.write)
			{
				msn_table[rxRequest.qpn].msn = rxRequest.msn;
				msn_table[rxRequest.qpn].vaddr = rxRequest.vaddr;
				msn_table[rxRequest.qpn].dma_length = rxRequest.dma_length;
			}
			else
			{
				msnTable2rxExh_rsp[i].write(dmaState(msn_table[rxRequest.qpn]));
			}
		}
		else if (!txExh2msnTable_req[i].empty())
		{
			txExh2msnTable_req[i].read(qpn);
			msnTable2txExh_rsp[i].write(txMsnRsp(msn_table[qpn].msn, msn_table[qpn].r_key));
		}
		else if (!if2msnTable_init[i].empty()) // move up??
		{
			std::cout << "MSN init for QPN: " << qpn << std::endl;
			if2msnTable_init[i].read(ifRequest);
			msn_table[ifRequest.qpn].msn = 0;
			msn_table[ifRequest.qpn].vaddr = 0;		 // TODO requried?
			msn_table[ifRequest.qpn].dma_length = 0; // TODO requried?
			msn_table[ifRequest.qpn].r_key = ifRequest.r_key;
		}
	}
}
