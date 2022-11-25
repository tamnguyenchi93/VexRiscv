class SpiIo {
	public:
		CData *CLK;
		CData *CS;
		CData *MISO;
		CData *MOSI;
};



class Spi : public SimElement {
	#define BUFF_LEN 16
  public:
	SpiIo *io;
	uint8_t curr_bit;
	uint8_t curr_byte;
	uint8_t rx_buff[BUFF_LEN];
	uint8_t tx_buff[BUFF_LEN];
	uint8_t tx_byte, rx_byte;

    bool is_return = false;
    uint8_t n_tx;
	uint8_t ckeLast = 0;
	Spi(SpiIo* io) {
		this->io = io;
	}
	virtual void preCycle(){
		if (!*io->CS && *io->CLK) {
			if (!ckeLast) {
                *io->MISO = (tx_byte >> (7 - curr_bit)) & 0x01;
                rx_byte |= ((*io->MOSI & 0x01)<< (7 - curr_bit));
				curr_bit++;
				if (curr_bit == 8) {
					curr_bit = 0;
                    // Do receive byte logic
                    receive_byte();
				}
			}
		}
		if (*io->CS) {
			reset();
		}
		ckeLast = *io->CLK;
	}
	virtual void receive_byte(){}
	void reset() {
        rx_byte = 0;
        tx_byte = 0;
		this->curr_bit = 0;
		this->curr_byte = 0;
		memset(rx_buff, 0, sizeof(rx_buff)/sizeof(rx_buff[0]));
	}
};