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

#include <iostream>
#include "../axi_utils.hpp"
#include "../mem_utils.hpp"
#include "../packet.hpp"
#include "../ipv6/ipv6.hpp"
#include "../udp/udp.hpp"
// #include "conn_table.hpp"
#include "state_table.hpp"
#include "msn_table.hpp"
#include "transport_timer.hpp"
#include "retransmitter/retransmitter.hpp"
#include "read_req_table.hpp"
#include "multi_queue/multi_queue.hpp"
#include "conn_table_entry.hpp"
#include "state_table_structs.hpp"
#include "ib_types.hpp"
using namespace hls;

const uint32_t BTH_SIZE = 96;
const uint32_t RETH_SIZE = 128;
const uint32_t AETH_SIZE = 32;
const uint32_t IMMDT_SIZE = 32;
const uint32_t RPCH_SIZE = 224;
const uint32_t ATOMICTH_SIZE = 28;
const uint32_t ATOMICACKH_SIZE = 8;

const ap_uint<16> RDMA_DEFAULT_PORT = 0x12B7; //4791 --> 0x12B7

#define RETRANS_EN 0


bool checkIfResponse(ibOpCode code);
bool checkIfWriteOrPartReq(ibOpCode code);
bool checkIfAethHeader(ibOpCode code);
bool checkIfRethHeader(ibOpCode code);


//TODO clean this up, these things seem to cover similar information
struct pkgSplitType
{
	ibOpCode op_code;
	axiRoute route;
	pkgSplitType() {}
	pkgSplitType(ibOpCode op)
			:op_code(op), route(ROUTE_DMA) {}
	pkgSplitType(ibOpCode op, axiRoute route)
		:op_code(op), route(route) {}
};

struct pkgInfo
{
	pkgType		type;
	pkgSource	source;
	ap_uint<29> words;
	pkgInfo() {}
	pkgInfo(pkgType t, pkgSource src, ap_uint<29> words)
		:type(t), source(src), words(words) {}
};

//TODO add command?
struct qpContext
{
	qpState		newState;
	ap_uint<24> qp_num;
	ap_uint<24> remote_psn;
	ap_uint<24> local_psn;
	ap_uint<16> r_key;
	ap_uint<48> virtual_address;
};

struct 	ifConnReq
{
	ap_uint<16> qpn;
	ap_uint<24> remote_qpn;
	ap_uint<128> remote_ip_address; //TODO make variable
	ap_uint<16> remote_udp_port; //TODO what is this used for
};

struct readRequest
{
	ap_uint<24> qpn;
	ap_uint<48> vaddr;
	ap_uint<32> dma_length;
	ap_uint<24> psn;
//#if POINTER_CHASING_EN
	axiRoute	route;
//#endif
	readRequest() {}
	readRequest(ap_uint<24> qpn, ap_uint<48> vaddr, ap_uint<32> len, ap_uint<24> psn)
//#if !POINTER_CHASING_EN
//		:qpn(qpn), vaddr(vaddr), dma_length(len), psn(psn) {}
//#else
		:qpn(qpn), vaddr(vaddr), dma_length(len), psn(psn), route(ROUTE_DMA) {}
	readRequest(ap_uint<24> qpn, ap_uint<48> vaddr, ap_uint<32> len, ap_uint<24> psn, axiRoute route)
			:qpn(qpn), vaddr(vaddr), dma_length(len), psn(psn), route(route) {}
//#endif
};

struct fwdPolicy
{
	bool isDrop;
	bool ackOnly;
	fwdPolicy()
		:isDrop(false), ackOnly(false) {}
	fwdPolicy(bool drop, bool ack)
		:isDrop(drop), ackOnly(ack) {}
};

//TODO move to UDP/IP
struct dstTuple
{
	ap_uint<128> their_address;
	ap_uint<16> their_port;
	dstTuple() {}
	dstTuple(ap_uint<128> addr, ap_uint<16> port)
		:their_address(addr), their_port(port) {}
};

struct txPacketInfo
{
	bool isAETH;
	bool hasHeader;
	bool hasPayload;
};

typedef enum {APP_READ, APP_WRITE, APP_PART, APP_POINTER, APP_READ_CONSISTENT, APP_ATOMIC_ADD} appOpCode;
//typedef enum {ORIGIN_FPGA, ORIGIN_HOST} reqOrigin;

struct txMeta
{
	appOpCode 	op_code;
	//reqOrigin	origin;
	ap_uint<24> qpn;
	ap_uint<48> local_vaddr;
	ap_uint<48> remote_vaddr;
	ap_uint<32> length;
	txMeta()
		:op_code(APP_READ) {}
	txMeta(appOpCode op, ap_uint<24> qp, ap_uint<48> raddr, ap_uint<32> len)
				:op_code(op), qpn(qp), local_vaddr(0), remote_vaddr(raddr), length(len){}
	txMeta(appOpCode op, ap_uint<24> qp, ap_uint<48> laddr, ap_uint<48> raddr, ap_uint<32> len)
			:op_code(op), qpn(qp), local_vaddr(laddr), remote_vaddr(raddr), length(len){}
};


struct ackEvent
{
	ap_uint<24> qpn;
	ap_uint<24>	psn;
	bool		validPsn;
	bool		isNak;
	ackEvent() {}
	ackEvent(ap_uint<24> qpn)
		:qpn(qpn), psn(0), validPsn(false), isNak(false) {}
	ackEvent(ap_uint<24> qpn, bool nak)
			:qpn(qpn), psn(0), validPsn(false), isNak(nak) {}
	ackEvent(ap_uint<24> qp, ap_uint<24> psn, bool nak)
			:qpn(qp), psn(psn), validPsn(true), isNak(nak) {}
};

//TODO create readEvent
//TODO event for writes addr + len, no psn
struct event
{
	ibOpCode 	op_code;
	ap_uint<24> qpn;
	ap_uint<48> addr;
	ap_uint<32> length;
	ap_uint<24>	psn;
	bool		validPsn;
	bool		isNak;
	event()
		:op_code(RC_ACK), validPsn(false), isNak(false) {}
	event(ibOpCode op, ap_uint<24> qp)
		:op_code(op), qpn(qp), validPsn(false), isNak(false) {}
	event(ackEvent& aev)
		:op_code(RC_ACK), qpn(aev.qpn), psn(aev.psn), validPsn(aev.validPsn), isNak(aev.isNak) {}
	/*event(ibOpCode op, ap_uint<24> qp, ap_uint<24> psn, bool nak)
		:op_code(op), qpn(qp), psn(psn), validPsn(true), isNak(nak) {}*/
	event(ibOpCode op, ap_uint<24> qp, ap_uint<48> addr, ap_uint<32> len)
		:op_code(op), qpn(qp), addr(addr), length(len), psn(0), validPsn(false), isNak(false) {}
	event(ibOpCode op, ap_uint<24> qp, ap_uint<32> len, ap_uint<24> psn)
		:op_code(op), qpn(qp), addr(0), length(len), psn(psn), validPsn(true), isNak(false) {}
	event(ibOpCode op, ap_uint<24> qp, ap_uint<48> addr, ap_uint<32> len, ap_uint<24> psn)
		:op_code(op), qpn(qp), addr(addr), length(len), psn(psn), validPsn(true), isNak(false) {}
};

struct retransEvent
{
	ibOpCode 	op_code;
	ap_uint<24> qpn;
	ap_uint<48> localAddr;
	ap_uint<48> remoteAddr;
	ap_uint<32> length;
	ap_uint<24>	psn;
	bool		validPsn;  //TODO remove?
	bool		isNak; //TODO remove?
	retransEvent()
		:op_code(RC_ACK), validPsn(false), isNak(false) {}
	/*event(ibOpCode op, ap_uint<24> qp)
		:op_code(op), qpn(qp), validPsn(false), isNak(false) {}
	event(ibOpCode op, ap_uint<24> qp, ap_uint<24> psn, bool nak)
		:op_code(op), qpn(qp), psn(psn), validPsn(true), isNak(nak) {}
	event(ibOpCode op, ap_uint<24> qp, ap_uint<48> addr, ap_uint<32> len)
		:op_code(op), qpn(qp), addr(addr), length(len), psn(0), validPsn(false), isNak(false) {}
	event(ibOpCode op, ap_uint<24> qp, ap_uint<32> len, ap_uint<24> psn)
		:op_code(op), qpn(qp), addr(0), length(len), psn(psn), validPsn(true), isNak(false) {}*/
	retransEvent(ibOpCode op, ap_uint<24> qp, ap_uint<48> laddr, ap_uint<48> raddr, ap_uint<32> len, ap_uint<24> psn)
		:op_code(op), qpn(qp), localAddr(laddr), remoteAddr(raddr),length(len), psn(psn), validPsn(true), isNak(false) {}
};


/*struct ackEvent// : public event
{
	ackEvent() {}
	ackEvent(const event& e)
		:ev(e.op_code, e.qpn, e.addr, e.length) {}
	ackEvent(ap_uint<24> qp)
		:ev(RC_ACK, qp, 0, 0) {}
	ackEvent(ap_uint<24> qp, ap_uint<24> psn, bool nak)
		:ev(RC_ACK, qp, psn, nak) {} //TODO
public:
	event getEvent() const
	{
		return ev;
	}
	ap_uint<24> getPSN()
	{
		return ev.addr(23, 0);
	}
	bool isNak()
	{
		return (ev.length != 0);
	}
private:
	event ev;
};

struct eventPSN// : public event
{
	ap_uint<24> psn;
	eventPSN () {}
	eventPSN(const event& e)
		:ev(e.op_code, e.qpn, e.addr, e.length) {}
	eventPSN(const event& ev, ap_uint<24> psn)
		:ev(ev.op_code, ev.qpn, ev.addr, ev.length), psn(psn) {}
	eventPSN(ibOpCode op, ap_uint<24> qp, ap_uint<32> len, ap_uint<24> psn)
		:ev(op, qp, 0, len), psn(psn) {}
	eventPSN(ibOpCode op, ap_uint<24> qp, ap_uint<64> addr, ap_uint<32> len, ap_uint<24> psn)
		:ev(op, qp, addr, len), psn(psn) {}
	event& getEvent()
	{
		return ev;
	}
private:
	event ev;
};*/

struct memMeta
{
	bool write;
	ap_uint<48> address;
	ap_uint<32> length;
	memMeta() {}
	memMeta(bool w, ap_uint<48> addr, ap_uint<32> len)
		:write(w), address(addr), length(len) {}
};


/**
 * BaseTransportHeader
 * header(7, 0) = op_code; // OpCode (8bit), IBA packet type, which extension headers follow
 * header[8] = solicited_event;
 * header[9] = mig_req;
 * header(11,10) = pad_count;
 * header(15, 12) = t_ver;
 * header(31, 16) = partition_key;
 * header[32] = fr;
 * header[33] = br;
 * header(39, 34) = reserved_var;
 * header(63, 40) = reverse(dest_qp);
 * header[71] = ack_req;
 * header(70, 64) = reserved;
 * header(95, 72) = reverse(psn);
 */

template <int W>
class BaseTransportHeader : public packetHeader<W, BTH_SIZE> {
	using packetHeader<W, BTH_SIZE>::header;

public:
	BaseTransportHeader() {} //TODO set initialize some fields

	void setOpCode(ibOpCode opcode)
	{
		header(7, 0) =  (uint8_t) opcode;
	}
	ibOpCode getOpCode()
	{
		return ibOpCode((uint16_t) header(7, 0));
	}
	void setPartitionKey(ap_uint<16> key)
	{
		header(31, 16) = key;
	}
	ap_uint<16> getPartitionKey()
	{
		return header(31, 16);
	}
	void setPsn(ap_uint<24> psn)
	{
		header(95, 72) = reverse(psn);
	}
	ap_uint<24> getPsn()
	{
		return reverse((ap_uint<24>) header(95, 72));
	}
	void setDstQP(ap_uint<24> qp)
	{
		header(63, 40) = reverse(qp);
	}
	ap_uint<24> getDstQP()
	{
		return reverse((ap_uint<24>) header(63, 40));
	}
	void setAckReq(bool bit)
	{
		header[71] = bit;
	}
};

struct ibhMeta
{
	ibOpCode op_code;
	ap_uint<16> partition_key;
	ap_uint<24> dest_qp;
	ap_uint<24> psn;
	bool		validPSN;
	ap_uint<22> numPkg; //TODO does not really fit here //TODO how many bits does this need?
	ibhMeta()
		:op_code(RC_ACK) {} //TODO
	ibhMeta(ibOpCode op, ap_uint<16> key, ap_uint<24> qp)
			:op_code(op), partition_key(key), dest_qp(qp), psn(0), validPSN(false), numPkg(1) {}
	ibhMeta(ibOpCode op, ap_uint<16> key, ap_uint<24> qp, ap_uint<22> numPkg)
			:op_code(op), partition_key(key), dest_qp(qp), psn(0), validPSN(false), numPkg(numPkg) {}
	ibhMeta(ibOpCode op, ap_uint<16> key, ap_uint<24> qp, ap_uint<24> psn, bool vp)
			:op_code(op), partition_key(key), dest_qp(qp), psn(psn), validPSN(vp), numPkg(1) {}
};

struct exhMeta
{
	bool		isNak;
	ap_uint<22> numPkg; //TODO how many bits does this need?
	exhMeta() {}
	exhMeta(bool isNak)
		:isNak(isNak), numPkg(1) {}
	exhMeta(bool isNak, ap_uint<22> numPkg)
		:isNak(isNak), numPkg(numPkg) {}
};


//TODO currently not used
/*struct DatagramExHeader //DETH
{
	ap_uint<32> queue_key; // required to authorize access to receive queue
	ap_uint<8> reserved;
	ap_uint<24> src_qp; // indicates QP at the source
	net_axis<WIDTH> createWord()
	{
		net_axis<WIDTH> w;
		w.data(31,0) = queue_key;
		w.data(39,32) = reserved;
		w.data(63, 40) = src_qp;
		w.keep = 0x000000FF;
		w.last = 1;
		return w;
	}
	void parseWord(ap_uint<256>& w)
	{
		queue_key = w(31, 0);
		//reserved = w(39, 32);
		src_qp = w(64, 40);
	}
};*/

/**
 *  see page 167
 *	ap_uint<64> virtual_address;
 *	ap_uint<32> r_key;
 *	ap_uint<32> dma_len;
 */
template <int W>
class RdmaExHeader : public packetHeader<W, RETH_SIZE>//RETH
{
	using packetHeader<W, RETH_SIZE>::header;

public:
	RdmaExHeader() {}

	void setVirtualAddress(ap_uint<64> addr) //TODO & or not??
	{
		header(63, 0) = reverse(addr); //TODO or reverseByte
	}
	ap_uint<64> getVirtualAddress()
	{
		return reverse((ap_uint<64>) header(63, 0));
	}
	void setRemoteKey(ap_uint<32> key)
	{
		header(95, 64) = reverse(key);
	}
	ap_uint<32> getRemoteKey()
	{
		return reverse((ap_uint<32>) header(95,64));
	}
	void setLength(ap_uint<32> len)
	{
		header(127,96) = reverse(len);
	}
	ap_uint<32> getLength()
	{
		return reverse((ap_uint<32>) header(127,96));
	}

};

template <int W>
class AtomicExHeader: public packetHeader<W, ATOMICTH_SIZE> {
	using packetHeader<W, ATOMICTH_SIZE>::header;
public:
	AtomicExHeader() {}

	void setVirtualAddress(ap_uint<64> addr) {
		header(64, 0) = reverse(addr);
	}
	ap_uint<64> getVirtualAddress() {
		return reverse((ap_uint<64>) header(63, 0));
	}

	void setRemoteKey(ap_uint<32> key) {
		header(95, 64);
	}

	ap_uint<32> getRemoteKey() {
		return reverse((ap_uint<32>) header(95,64));
	}
	void setSwapAddData(ap_uint<64> data) {
		header(160, 96) = reverse(data);
	}
	ap_uint<64> getSwapAddData() {
		return reverse((ap_uint<64>) header(160, 96));
	}
	void setCompareData(ap_uint<64> compare) {
		header(224, 161) = reverse(compare);
	}
	ap_uint<64> getCompareData() {
		return reverse((ap_uint<64>) header(224, 161));	
	}
};
template <int W>
class AtomicAckExHeader : public packetHeader<W, ATOMICACKH_SIZE> {
	using packetHeader<W, ATOMICACKH_SIZE>::header;
public:
	AtomicAckExHeader() {}

	void setOrigRemoteDt(ap_uint<64> data){
		header(64, 0) = reverse(data);		
	}

	ap_uint<64> getOrigRemoteDt() {
		return reverse((ap_uint<64>) header(64, 0));
	}

};

/**
 * AETH, see page 254, page format on 337, NAK codes page 338
 * [7:0] syndrome, indicates if this is an ACK or NAK
 * [31:8] msn, number of last message completed
 */
template <int W>
class AckExHeader : public packetHeader<W, AETH_SIZE> //AETH
{
	using packetHeader<W, AETH_SIZE>::header;

public:
	AckExHeader() {}

	void setSyndrome(ap_uint<8> syn)
	{
		header(7, 0) = syn;
	}
	ap_uint<8> getSyndrome()
	{
		return ((ap_uint<8>) header(7, 0));
	}
	void setMsn(ap_uint<24> msn)
	{
		header(31, 8) = reverse(msn);
	}
	ap_uint<24> getMsn()
	{
		return reverse((ap_uint<32>) header(31, 8));
	}
	bool isNAK()
	{
		return (header(6,5) == 0x3);
	}
};

/**
 *  Custom Header to do pointer chasing on the remote node
 *	ap_uint<64> virtual_address;
 *	ap_uint<32> r_key;
 *	ap_uint<32> dma_len;
 *  ap_uint<64> predicate_key;
 *  ap_uint<16> predicate_mask;
 *  ap_uint<2>  predicate_op;
 *  ap_uint<5>  ptr_offset;
 *  ap_uint<1>  is_relative_ptr;
 *  ap_uint<5>  next_ptr_offset;
 *  ap_uint<1>  next_valid;
 *  ap_uint<2>  reserved;
 */
#if POINTER_CHASING_EN
template <int W>
class RdmaPointerChaseHeader : public packetHeader<W, RPCH_SIZE>//RCTH
{
	using packetHeader<W, RPCH_SIZE>::header;

public:
	RdmaPointerChaseHeader() {}

	void setVirtualAddress(ap_uint<64> addr) //TODO & or not??
	{
		header(63, 0) = reverse(addr); //TODO or reverseByte
	}
	ap_uint<64> getVirtualAddress()
	{
		return reverse((ap_uint<64>) header(63, 0));
	}
	void setRemoteKey(ap_uint<32> key)
	{
		header(95, 64) = reverse(key);
	}
	ap_uint<32> getRemoteKey()
	{
		return reverse((ap_uint<32>) header(95,64));
	}
	void setLength(ap_uint<32> len)
	{
		header(127,96) = reverse(len);
	}
	ap_uint<32> getLength()
	{
		return reverse((ap_uint<32>) header(127,96));
	}
	void setPredicateKey(ap_uint<64> key)
	{
		header(191,128) = reverse(key);
	}
	ap_uint<64> getPredicateKey()
	{
		return reverse((ap_uint<64>) header(191,128));
	}
	void setPredicateMask(ap_uint<16> mask)
	{
		header(207,192) = reverse(mask);
	}
	ap_uint<16> getPredicateMask()
	{
		return reverse((ap_uint<16>) header(207,192));
	}
	void setPredicateOp(ap_uint<2> op)
	{
		header(215,214) = op;
	}
	predicateOp getPredicateOp()
	{
		return (predicateOp) (int) (ap_uint<2>)header(215,214);
	}
	void setPtrOffset(ap_uint<5> offset)
	{
		header(213,209) = offset;
	}
	ap_uint<5> getPtrOffset()
	{
		return header(213,209);
	}
	void setIsRelPtr(ap_uint<1> isRelPtr)
	{
		header[208] = isRelPtr;
	}
	ap_uint<1> getIsRelPtr()
	{
	return header[208];
	}
	void setNextPtrOffset(ap_uint<5> offset)
	{
		header(223,219) = offset;
	}
	ap_uint<5> getNextPtrOffset()
	{
		return header(223,219);
	}
	void setNexPtrValid(ap_uint<1> nextPtrValid)
	{
		header[218] = nextPtrValid;
	}
	ap_uint<1> getNextPtrValid()
	{
	return header[218];
	}
	//216, 217 reserverd
};
#endif

template <int W>
class ExHeader: public packetHeader<W, RPCH_SIZE>
{
	using packetHeader<W, RPCH_SIZE>::header;

public:
	ExHeader() {}

	ExHeader(RdmaExHeader<W>& h)
	{
		header = h.getRawHeader();
	}
	ExHeader(AckExHeader<W>& h)
	{
		header = h.getRawHeader();
	}
	ExHeader(AtomicExHeader<W>& h){
		header = h.getRawHeader();
	}
	/*ExHeader(RdmaPointerChaseHeader<W>& h)
	{
		header = h.getRawHeader();
	}*/
	RdmaExHeader<W> getRdmaHeader()
	{
		RdmaExHeader<W> rethHeader;
		rethHeader.setRawHeader(header);
		return rethHeader;
	}

	AckExHeader<W> getAckHeader()
	{
		AckExHeader<W> aethHeader;
		aethHeader.setRawHeader(header);
		return aethHeader;
	}

	AtomicExHeader<W> getAtomicHeader() 
	{
		AtomicExHeader<W> atomicHeader;
		atomicHeader.setRawHeader(header);
		return atomicHeader;
	}

	/*RdmaPointerChaseHeader<W> getPointerChasingHeader()
	{
		RdmaPointerChaseHeader<W> pcHeader;
		pcHeader.setRawHeader(header);
		return pcHeader;
	}*/
};


struct ImmDt
{
	ap_uint<32> immdt;
};

struct InvalidateExHeader //IETH
{
	ap_uint<32> r_key;
};

struct memCmdInternal
{
	ap_uint<16> qpn; //TODO required
	ap_uint<64> addr;
	ap_uint<32> len;
	//ap_uint<1>	last;
//#if POINTER_CHASING_EN
	axiRoute route;
//#endif@
	memCmdInternal() {}
	memCmdInternal(ap_uint<16> qpn, ap_uint<64> addr, ap_uint<32> len)
#if !POINTER_CHASING_EN
		:qpn(qpn), addr(addr), len(len) {}
#else
		:qpn(qpn), addr(addr), len(len), route(ROUTE_DMA) {}
	memCmdInternal(ap_uint<16> qpn, ap_uint<64> addr, ap_uint<32> len, axiRoute route)
		:qpn(qpn), addr(addr), len(len), route(route) {}
#endif
};

template <int WIDTH>
void ib_transport_protocol(	//RX
							hls::stream<ipUdpMeta>&	s_axis_rx_meta,
							hls::stream<net_axis<WIDTH> >&	s_axis_rx_data,
							//hls::stream<net_axis<WIDTH> >&	m_axis_rx_data,
							//TX
							hls::stream<txMeta>&	s_axis_tx_meta,
							hls::stream<net_axis<WIDTH> >&	s_axis_tx_data,
							hls::stream<ipUdpMeta>&	m_axis_tx_meta,
							hls::stream<net_axis<WIDTH> >&	m_axis_tx_data,
							//Memory
							hls::stream<routedMemCmd>&		m_axis_mem_write_cmd,
							hls::stream<routedMemCmd>&		m_axis_mem_read_cmd,
							hls::stream<routed_net_axis<WIDTH> >&	m_axis_mem_write_data,
							hls::stream<net_axis<WIDTH> >&	s_axis_mem_read_data,

							hls::stream<qpContext>&	s_axis_qp_interface,
							hls::stream<ifConnReq>&	s_axis_qp_conn_interface,

							//pointer chasing
#if POINTER_CHASING_EN
							hls::stream<ptrChaseMeta>&	m_axis_rx_pcmeta,
							hls::stream<ptrChaseMeta>&	s_axis_tx_pcmeta,
#endif
							//debug
							ap_uint<32>&		regInvalidPsnDropCount,
							hls::stream<ap_uint<16> >& tx_ibhconnTable_req,
							hls::stream<connTableEntry>& tx_connTable2ibh_rsp,
							hls::stream<rxStateReq>& rxIbh2stateTable_upd_req,
							hls::stream<txStateReq>& txIbh2stateTable_upd_req,
							hls::stream<ifStateReq>& qpi2stateTable_upd_req,
							hls::stream<rxStateRsp>& stateTable2rxIbh_rsp,
							hls::stream<stateTableEntry>& stateTable2txIbh_rsp,
							hls::stream<stateTableEntry> &stateTable2qpi_rsp,
							hls::stream<rxMsnReq>& rxExh2msnTable_upd_req,
							hls::stream<ap_uint<16> >& txExh2msnTable_req,
							hls::stream<ifMsnReq>& if2msnTable_init,
							hls::stream<dmaState>& msnTable2rxExh_rsp,
							hls::stream<txMsnRsp>& msnTable2txExh_rsp,
							hls::stream<ap_uint<16> >& exh_lengthFifo,
							hls::stream<readRequest>& rx_readRequestFifo,
							hls::stream<event>& rx_readEvenFifo,
							hls::stream<ackEvent>& rx_ackEventFifo,
							hls::stream<txReadReqUpdate>& tx_readReqTable_upd,
							hls::stream<rxReadReqUpdate>& rx_readReqTable_upd_req,
							hls::stream<rxReadReqRsp>& rx_readReqTable_upd_rsp,
							hls::stream<mqInsertReq<ap_uint<64> > >& tx_readReqAddr_push,
							hls::stream<mqPopReq>& rx_readReqAddr_pop_req,
							hls::stream<ap_uint<64> >& rx_readReqAddr_pop_rsp
						);