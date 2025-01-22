#include "../axi_utils.hpp"
#include "ib_types.hpp"
#pragma once
//PSN, page 293, 307, 345
struct stateTableEntry
{
	//window
	ap_uint<24> resp_epsn;
	ap_uint<24> resp_old_outstanding;

	ap_uint<24> req_next_psn;
	ap_uint<24> req_old_unack;
	ap_uint<24> req_old_valid; //required? can be computed?
	ap_uint<3>	retryCounter;
};

struct ifStateReq
{
	ap_uint<16> qpn;
	qpState		newState;
	ap_uint<24> remote_psn;
	ap_uint<24> local_psn;
	bool		write;
	ifStateReq() {}
	ifStateReq(ap_uint<24> qpn)
		:qpn(qpn), write(false) {}
	ifStateReq(ap_uint<16> qpn, qpState s, ap_uint<24> rpsn, ap_uint<24> lpsn)
		:qpn(qpn), newState(s), remote_psn(rpsn), local_psn(lpsn), write(true) {}
};

struct rxStateReq
{
	ap_uint<16> qpn;
	ap_uint<24> epsn;
	ap_uint<3>	retryCounter;
	bool		isResponse;
	bool		write;
	rxStateReq() :isResponse(false), write(false) {}
	rxStateReq(ap_uint<24> qpn, bool isRsp)
		:qpn(qpn), isResponse(isRsp), write(false) {}
	rxStateReq(ap_uint<16> qpn, ap_uint<24> psn, bool isRsp)
		:qpn(qpn), epsn(psn), isResponse(isRsp), retryCounter(0x7), write(true) {}
	rxStateReq(ap_uint<16> qpn, ap_uint<24> psn, ap_uint<3> rc, bool isRsp)
		:qpn(qpn), epsn(psn), isResponse(isRsp), retryCounter(rc), write(true) {}
};


struct rxStateRsp
{
	//window
	ap_uint<24> epsn;
	ap_uint<24> oldest_outstanding_psn;
	ap_uint<24> max_forward; //used for reponses, page 346

	ap_uint<3>	retryCounter;
	rxStateRsp() {}
	rxStateRsp(ap_uint<24> epsn, ap_uint<24> old)
		:epsn(epsn), oldest_outstanding_psn(old), max_forward(0), retryCounter(0) {}
	rxStateRsp(ap_uint<24> epsn, ap_uint<24> old, ap_uint<24> maxf)
		:epsn(epsn), oldest_outstanding_psn(old), max_forward(maxf), retryCounter(0) {}
	rxStateRsp(ap_uint<24> epsn, ap_uint<24> old, ap_uint<24> maxf, ap_uint<3> rc)
		:epsn(epsn), oldest_outstanding_psn(old), max_forward(maxf), retryCounter(rc) {}
};

struct txStateReq
{
	ap_uint<16> qpn;
	ap_uint<24> psn;
	bool		write;
	txStateReq() :write(false) {}
	txStateReq(ap_uint<24> qpn)
		:qpn(qpn), write(false) {}
	txStateReq(ap_uint<16> qpn, ap_uint<24> psn)
		:qpn(qpn), psn(psn), write(true) {}
};
