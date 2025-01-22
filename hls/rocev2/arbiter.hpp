#include "../axi_utils.hpp"
#include "../ipv4/ipv4.hpp"
#include "../ipv6/ipv6.hpp"
#include "../udp/udp.hpp"

template<int WIDTH, int N>
void arbiter(
    stream<ipUdpMeta>& rx_ipUdpMetaFifo,
    stream<ipUdpMeta>& tx_ipUdpMetaFifo,
    stream<net_axis<WIDTH> >& rx_udp2ibFifo,
    stream<net_axis<WIDTH> >& tx_ib2udpFifo,
    stream<ipUdpMeta> rx_ipUdpMetaFifoArray[N],
    stream<ipUdpMeta> tx_ipUdpMetaFifoArray[N],
    stream<net_axis<WIDTH> > rx_udp2ibFifoArray[N],
    stream<net_axis<WIDTH> > tx_ib2udpFifoArray[N]
) {
// #pragma HLS STREAM depth = 2 variable = rx_ipUdpMetaFifo
// #pragma HLS STREAM depth = 2 variable = tx_ipUdpMetaFifo
// #pragma HLS STREAM depth = 2 variable = rx_crc2ipFifo
// #pragma HLS STREAM depth = 2 variable = rx_udp2ibFifo
// #pragma HLS STREAM depth = 2 variable = rx_crc2ipFifo
// #pragma HLS STREAM depth = 2 variable = rx_ip2udpFifo
// #pragma HLS STREAM depth = 2 variable = tx_udp2ipMetaFifo
// #pragma HLS STREAM depth = 2 variable = tx_udp2ipFifo

#pragma HLS ARRAY_PARTITION variable = rx_ipUdpMetaFifoArray dim = 1 complete
#pragma HLS ARRAY_PARTITION variable = tx_ipUdpMetaFifoArray dim = 1 complete
#pragma HLS ARRAY_PARTITION variable = rx_udp2ibFifoArray dim = 1 complete
#pragma HLS ARRAY_PARTITION variable = tx_ib2udpFifoArray dim = 1 complete

    static int chosen;

    // std::cout << "ARB: " << chosen << std::endl; 
    if(!rx_ipUdpMetaFifo.empty()){
        rx_ipUdpMetaFifoArray[chosen].write(rx_ipUdpMetaFifo.read());
    }
    for(int i = 0; i < N; i++){
        #pragma HLS unroll
        while(!tx_ipUdpMetaFifoArray[i].empty()){
            tx_ipUdpMetaFifo.write(tx_ipUdpMetaFifoArray[i].read());
        }
    }
    // if(!rx_crc2ipFifo.empty()){
    //     rx_crc2ipFifoArray[chosen].write(rx_crc2ipFifo.read());
    // }
    if(!rx_udp2ibFifo.empty()){
        rx_udp2ibFifoArray[chosen].write(rx_udp2ibFifo.read());
    }
    for (int i = 0; i < N; i++){
        #pragma HLS unroll
        while(!tx_ib2udpFifoArray[i].empty()){
            tx_ib2udpFifo.write(tx_ib2udpFifoArray[i].read());
        }
    }
    // if(!tx_ip2crcFifo.empty()){
    //     tx_ip2crcFifoArray[chosen].write(tx_ip2crcFifo.read());
    // }
    // if(!rx_ip2udpMetaFifo.empty()){
    //     rx_ip2udpMetaFifoArray[chosen].write(rx_ip2udpMetaFifo.read());
    // }
    // if(!rx_ip2udpFifo.empty()){
    //     rx_ip2udpFifoArray[chosen].write(rx_ip2udpFifo.read());
    // }
    // if(!tx_udp2ipMetaFifo.empty()){
    //     tx_udp2ipMetaFifoArray[chosen].write(tx_udp2ipMetaFifo.read());
    // }
    // if(!tx_udp2ipFifo.empty()){
    //     tx_udp2ipFifoArray[chosen].write(tx_udp2ipFifo.read());
    // }
    
    chosen++;

    if(chosen == N){
        chosen = 0;
    }

    

}