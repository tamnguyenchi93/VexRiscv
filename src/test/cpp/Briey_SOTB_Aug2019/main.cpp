#include "VBriey_SOTB_Aug2019.h"
#include "VBriey_SOTB_Aug2019_Briey_SOTB_Aug2019.h"
//#include "VBriey_Axi4VgaCtrl.h"
//#include "VBriey_VgaCtrl.h"
#ifdef REF
#include "VBriey_SOTB_Aug2019_RiscvCore.h"
#endif
#include "verilated.h"
#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <stdint.h>
#include <cstring>
#include <string.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <iomanip>
#include <time.h>
#include <unistd.h>

#include "VBriey_SOTB_Aug2019_VexRiscv.h"


#include "../common/framework.h"
#include "../common/jtag.h"
#include "../common/uart.h"


class SDCardSpiIo{
	public:
		CData *CLK;
		CData *CS;
		CData *MISO;
		CData *MOSI;
};

#define SD_ACK		0x00
#define SD_IDLE		0x01
#define SD_PATTERN	0x000001aa
#define SD_CCS		0x40000000
#define SD_WAIT		0x80
#define SD_INVALID	0xff
typedef enum SD_STATE {
	SD_STATE_IDLE,
	SD_STATE_CMD0,
	SD_STATE_CMD8,
	SD_STATE_CMD58,
	SD_STATE_CMD16,
	SD_STATE_CMD41,
	SD_STATE_CMD55,
	SD_STATE_RESPOND,
} SD_state;

class SDCardSpi : public SimElement {
	#define MAX_CMD_BYTE 6
	public:
		SDCardSpiIo *io;
		uint8_t curr_bit;
		uint8_t curr_byte;
		uint8_t rx_buff[MAX_CMD_BYTE];
        uint8_t tx_buff[MAX_CMD_BYTE];
	
    uint8_t n_tx;
	SD_state sd_state;
	SD_state sd_lst_state;
    bool is_return = false;
	uint8_t ckeLast = 0;
    
	uint8_t tx_byte, rx_byte;
	
	SDCardSpi(SDCardSpiIo* io) {
		this->io = io;
		sd_state = SD_STATE_IDLE;
		reset();
	}
	virtual ~SDCardSpi(){
		;
	}

	virtual void postCycle(){
	}

	virtual void preCycle(){
		if (!*io->CS && *io->CLK) {
			if (!ckeLast) {
                *io->MISO = (tx_byte >> (7 - curr_bit)) & 0x01;
                rx_byte |= ((*io->MOSI & 0x01)<< (7 - curr_bit));
				curr_bit++;
				if (curr_bit == 8) {
					curr_bit = 0;
                    // Do SDCard logic
                    do_something();
				}
			}
		}
		if (*io->CS) {
			reset();
		}
		ckeLast = *io->CLK;
	}

    void do_something() {
        uint8_t cmd;
        uint32_t pattern;
        rx_buff[curr_byte] = rx_byte;
        rx_byte = 0;
        if (is_return == true && curr_byte == n_tx - 1) {
            curr_byte = 0;
            is_return = false;
        } else {
            curr_byte++;
        }

        if (curr_byte == MAX_CMD_BYTE) {
            curr_byte = 0;
            cmd = rx_buff[0] & ~0x40;
            // printf("cmd: %d\n",cmd);
            switch (sd_state)
            {
            case SD_STATE_IDLE:
                if (cmd == 0)
                    tx_buff[0] = SD_IDLE;
                else
                    tx_buff[0] = SD_INVALID;
                n_tx = 1;
                sd_state = SD_STATE_CMD0;
                is_return = true;
                break;
            case SD_STATE_CMD0:
                if (cmd == 8) {
                    pattern = rx_buff[1]<<24 | rx_buff[2]<<16 | rx_buff[3]<<8 | rx_buff[4];
                    printf("Receive pattern 0x%04X\n", pattern);
                    tx_buff[0] = SD_IDLE;
                    tx_buff[1] = rx_buff[1];
                    tx_buff[2] = rx_buff[2];
                    tx_buff[3] = rx_buff[3];
                    tx_buff[4] = rx_buff[4];
                    sd_state = SD_STATE_CMD8;
                    n_tx = 5;
                } else {
                    tx_buff[0] = SD_INVALID;
                    sd_state = SD_STATE_IDLE;
                    n_tx = 1;
                }
                is_return = true;
            case SD_STATE_CMD8:
                if (cmd == 55) {
                    tx_buff[0] = SD_IDLE;
                    sd_state = SD_STATE_CMD55;
                } else {
                    tx_buff[0] = SD_INVALID;
                    sd_state = SD_STATE_IDLE;
                }
                n_tx = 1;
                is_return = true;
            case SD_STATE_CMD55:
                if (cmd == 41) {
                    tx_buff[0] = SD_ACK;
                    sd_state = SD_STATE_CMD41;
                } else {
                    tx_buff[0] = SD_INVALID;
                    sd_state = SD_STATE_IDLE;
                }
                n_tx = 1;
            case SD_STATE_CMD41:
                if (cmd == 58) {
                    tx_buff[0] = SD_ACK;
                    sd_state = SD_STATE_CMD58;
                    tx_buff[0] = 0xc0;
                    tx_buff[1] = rx_buff[1];
                    tx_buff[2] = rx_buff[2];
                    tx_buff[3] = rx_buff[3];
                    n_tx = 4;
                } else {
                    tx_buff[0] = SD_INVALID;
                    sd_state = SD_STATE_IDLE;
                    n_tx = 1;
                }
            case SD_STATE_CMD58:
                if (cmd == 16) {
                    tx_buff[0] = SD_ACK;
                    sd_state = SD_STATE_CMD16;
                } else {
                    tx_buff[0] = SD_INVALID;
                    sd_state = SD_STATE_IDLE;
                }
                n_tx = 1;
            case SD_STATE_CMD16:
            default:
                break;
            }
        }
        tx_byte = tx_buff[curr_byte];
        printf("Next tx byte %d curr_byte %d\n", tx_byte, curr_byte);
    }
	void reset() {
        rx_byte = 0;
        tx_byte = 0;
		this->curr_bit = 0;
		this->curr_byte = 0;
		memset(rx_buff, 0, sizeof(rx_buff)/sizeof(rx_buff[0]));
	}
};

class SdramConfig{
public:
	uint32_t byteCount;
	uint32_t bankCount;
	uint32_t rowSize;
	uint32_t colSize;

	SdramConfig(uint32_t byteCount,
			uint32_t bankCount,
			uint32_t rowSize,
			uint32_t colSize){
		this->byteCount = byteCount;
		this->bankCount = bankCount;
		this->rowSize = rowSize;
		this->colSize = colSize;
	}
};

class SdramIo{
public:
	CData *BA;
	CData *DQM;
	CData *CASn;
	CData *CKE;
	CData *CSn;
	CData *RASn;
	CData *WEn;
	SData *ADDR;
	CData *DQ_read;
	CData *DQ_write;
	CData *DQ_writeEnable;
};

class Sdram : public SimElement{
public:

	SdramConfig *config;
	SdramIo *io;

	uint32_t CAS;
	uint32_t burstLength;

	class Bank{
	public:
		uint8_t *data;
		SdramConfig *config;

		bool opened;
		uint32_t openedRow;
		void init(SdramConfig *config){
			this->config = config;
			data = new uint8_t[config->rowSize * config->colSize * config->byteCount];
			opened = false;
		}

		virtual ~Bank(){
			delete data;
		}

		void activate(uint32_t row){
			if(opened)
				cout << "SDRAM error open unclosed bank" << endl;
			openedRow = row;
			opened = true;
		}

		void precharge(){
			opened = false;
		}

		void write(uint32_t column, CData byteId, CData data){
			if(!opened)
				cout << "SDRAM : write in closed bank" << endl;
			uint32_t addr = byteId + (column + openedRow * config->colSize) * config->byteCount;
			//printf("SDRAM : Write A=%08x D=%02x\n",addr,data);
			this->data[addr] = data;

		}

		CData read(uint32_t column, CData byteId){
			if(!opened)
				cout << "SDRAM : write in closed bank" << endl;
			uint32_t addr = byteId + (column + openedRow * config->colSize) * config->byteCount;
			//printf("SDRAM : Read A=%08x D=%02x\n",addr,data[addr]);
			return data[addr];
		}
	};

	Bank* banks;

	CData * readShifter;

	Sdram(SdramConfig *config,SdramIo* io){
		this->config = config;
		this->io = io;
		banks = new Bank[config->bankCount];
		for(uint32_t bankId = 0;bankId < config->bankCount;bankId++) banks[bankId].init(config);
		readShifter = new CData[config->byteCount*3];
	}

	virtual ~Sdram(){
		delete banks;
		delete readShifter;
	}


	uint8_t ckeLast = 0;


	virtual void postCycle(){
		if(CAS >= 2 && CAS <=3){
			for(uint32_t byteId = 0;byteId != config->byteCount;byteId++){
				io->DQ_read[byteId] = readShifter[byteId + (CAS-1)*config->byteCount];
			}
			for(uint32_t latency = CAS-1;latency != 0;latency--){  //missing CKE
				for(uint32_t byteId = 0;byteId != config->byteCount;byteId++){
					readShifter[byteId+latency*config->byteCount] = readShifter[byteId+(latency-1)*config->byteCount];
				}
			}
		}
	}

	virtual void preCycle(){
		if(!*io->CSn && ckeLast){
			uint32_t code = ((*io->RASn) << 2) | ((*io->CASn) << 1) | ((*io->WEn) << 0);
			switch(code){
			case 0: //Mode register set
				if(*io->BA == 0 && (*io->ADDR & 0x400) == 0){
					CAS = ((*io->ADDR) >> 4) & 0x7;
					burstLength = ((*io->ADDR) >> 0) & 0x7;
					if((*io->ADDR & 0x388) != 0)
						cout << "SDRAM : ???" << endl;
					printf("SDRAM : MODE REGISTER DEFINITION CAS=%d burstLength=%d\n",CAS,burstLength);
				}
				break;
			case 2: //Bank precharge
				if((*io->ADDR & 0x400) != 0){ //all
					for(uint32_t bankId = 0;bankId < config->bankCount;bankId++)
						banks[bankId].precharge();
				} else { //single
					banks[*io->BA].precharge();
				}
				break;
			case 3: //Bank activate
				banks[*io->BA].activate(*io->ADDR & 0x7FF);
				break;
			case 4: //Write
				if((*io->ADDR & 0x400) != 0)
					cout << "SDRAM : Write autoprecharge not supported" << endl;

				if(*io->DQ_writeEnable == 0)
					cout << "SDRAM : Write Wrong DQ direction" << endl;

				for(uint32_t byteId = 0;byteId < config->byteCount;byteId++){
					if(((*io->DQM >> byteId) & 1) == 0)
						banks[*io->BA].write(*io->ADDR, byteId ,io->DQ_write[byteId]);
				}
				break;

			case 5: //Read
				if((*io->ADDR & 0x400) != 0)
					cout << "SDRAM : READ autoprecharge not supported" << endl;

				if(*io->DQ_writeEnable != 0)
					cout << "SDRAM : READ Wrong DQ direction" << endl;

				//if(*io->DQM !=  config->byteCount-1)
					//cout << "SDRAM : READ wrong DQM" << endl;

				for(uint32_t byteId = 0;byteId < config->byteCount;byteId++){
					readShifter[byteId] = banks[*io->BA].read(*io->ADDR, byteId);
				}
				break;
			case 1: // Self refresh
				break;
			case 7: // NOP
				break;
			default:
				cout << "SDRAM : unknown code" << endl;
				break;
			}
		}
		ckeLast = *io->CKE;
	}
};


class VexRiscvTracer : public SimElement{
public:
	VBriey_SOTB_Aug2019_VexRiscv *cpu;
	ofstream instructionTraces;
	ofstream regTraces;

	VexRiscvTracer(VBriey_SOTB_Aug2019_VexRiscv *cpu){
		this->cpu = cpu;
#ifdef TRACE_INSTRUCTION
	instructionTraces.open ("instructionTrace.log");
#endif
#ifdef TRACE_REG
	regTraces.open ("regTraces.log");
#endif
	}



	virtual void preCycle(){
#ifdef TRACE_INSTRUCTION
		if(cpu->writeBack_arbitration_isFiring){
			instructionTraces <<  hex << setw(8) <<  cpu->writeBack_INSTRUCTION << endl;
		}
#endif
#ifdef TRACE_REG
		if(cpu->writeBack_RegFilePlugin_regFileWrite_valid == 1 && cpu->writeBack_RegFilePlugin_regFileWrite_payload_address != 0){
			regTraces << " PC " << hex << setw(8) <<  cpu->writeBack_PC << " : reg[" << dec << setw(2) << (uint32_t)cpu->writeBack_RegFilePlugin_regFileWrite_payload_address << "] = " << hex << setw(8) << cpu->writeBack_RegFilePlugin_regFileWrite_payload_data << endl;
		}

#endif
	}
};




#include <SDL2/SDL.h>
#include <assert.h>
#include <stdint.h>
#include <stdlib.h>


class Display : public SimElement{
public:
	int width, height;
	uint32_t *pixels;
    SDL_Window* window;
    SDL_Renderer* renderer;
    SDL_Texture * texture;
    uint32_t x,y;
    uint32_t refreshCounter = 0;

    Display(int width, int height){
		this->width = width;
		this->height = height;
		x = y = 0;
		init();
	}

	virtual ~Display(){
	    delete[] pixels;
	    SDL_DestroyTexture(texture);
	    SDL_DestroyRenderer(renderer);
	    SDL_DestroyWindow(window);
	    SDL_Quit();
	}

	void init(){

        /* Initialize SDL. */
        if (SDL_Init(SDL_INIT_VIDEO) < 0)
                return;

        /* Create the window where we will draw. */
        window = SDL_CreateWindow("VGA",
                        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
                        width, height,
                        SDL_WINDOW_SHOWN);

        /* We must call SDL_CreateRenderer in order for draw calls to affect this window. */
        renderer = SDL_CreateRenderer(window, -1, 0);

        texture = SDL_CreateTexture(renderer,
            SDL_PIXELFORMAT_ARGB8888, SDL_TEXTUREACCESS_STATIC, width, height);
        pixels = new Uint32[width * height];
        memset(pixels, 0, width * height * sizeof(Uint32));
	}

	void set(uint32_t color){
		pixels[x + y*width] = color;
	}

	void incX(){
		x++;
		if(x >= width) x = width;
	}

	void incY(){
		y++;
		if(y >= height) y = height;
	}

	void refresh(){
		//cout << "Display refresh " << refreshCounter++ << endl;
		SDL_UpdateTexture(texture, NULL, pixels, 640 * sizeof(Uint32));
		SDL_RenderClear(renderer);
		SDL_RenderCopy(renderer, texture, NULL, NULL);
		SDL_RenderPresent(renderer);
        memset(pixels, 0, width * height * sizeof(Uint32));
	}

	virtual void postCycle(){

	}

	virtual void preCycle(){

	}
};

#ifdef VGA
class Vga : public Display{
public:
	VBriey_SOTB_Aug2019* top;
	Vga(VBriey_SOTB_Aug2019* top,int width, int height) : Display(width, height){
		this->top = top;
	}

	virtual ~Vga(){
	}

	virtual void postCycle(){

	}

	uint32_t lastvSync = 0,lasthSync = 0;
	virtual void preCycle(){
		if(!top->io_vga_vSync && lastvSync) {
			y = 0;
			refresh();
		}
		if(!top->io_vga_hSync && lasthSync && x != 0) {
			incY();
			x = 0;
		}
		if(top->io_vga_colorEn){
			this->set((top->io_vga_color_r << 19) + (top->io_vga_color_g << 10) + (top->io_vga_color_b << 3));
			incX();
		}

		lastvSync = top->io_vga_vSync;
		lasthSync = top->io_vga_hSync;
	}
};

#endif

class BrieyWorkspace : public Workspace<VBriey_SOTB_Aug2019>{
public:
	BrieyWorkspace(uint32_t io_gpioA_read) : Workspace("Briey"){
		ClockDomain *axiClk = new ClockDomain(&top->io_axiClk,NULL,20000,100000);
        #ifdef VGA
		ClockDomain *vgaClk = new ClockDomain(&top->io_vgaClk,NULL,40000,100000);
        #endif
		AsyncReset *asyncReset = new AsyncReset(&top->io_asyncReset,50000);
		Jtag *jtag = new Jtag(&top->io_jtag_tms,&top->io_jtag_tdi,&top->io_jtag_tdo,&top->io_jtag_tck,80000);
		UartRx *uartRx = new UartRx(&top->io_uart_txd,1.0e12/115200);
		UartTx *uartTx = new UartTx(&top->io_uart_rxd,1.0e12/115200);
		timeProcesses.push_back(axiClk);
        #ifdef VGA
		timeProcesses.push_back(vgaClk);
        #endif
		timeProcesses.push_back(asyncReset);
		timeProcesses.push_back(jtag);
		timeProcesses.push_back(uartRx);
        timeProcesses.push_back(uartTx);

        top->io_gpioA_read = io_gpioA_read;
		SdramConfig *sdramConfig = new SdramConfig(
			2,  //byteCount
			4,  //bankCount
			1 << 13, //rowSize
			1 << 10  //colSize
		);
		// SdramIo *sdramIo = new SdramIo();
		// sdramIo->BA              = &top->io_sdram_BA             ;
		// sdramIo->DQM             = &top->io_sdram_DQM            ;
		// sdramIo->CASn            = &top->io_sdram_CASn           ;
		// sdramIo->CKE             = &top->io_sdram_CKE            ;
		// sdramIo->CSn             = &top->io_sdram_CSn            ;
		// sdramIo->RASn            = &top->io_sdram_RASn           ;
		// sdramIo->WEn             = &top->io_sdram_WEn            ;
		// sdramIo->ADDR            = &top->io_sdram_ADDR           ;
		// sdramIo->DQ_read         = (CData*)&top->io_sdram_DQ_read        ;
		// sdramIo->DQ_write        = (CData*)&top->io_sdram_DQ_write       ;
		// sdramIo->DQ_writeEnable = (CData*)&top->io_sdram_DQ_writeEnable;
		// Sdram *sdram = new Sdram(sdramConfig, sdramIo);

		// axiClk->add(sdram);

		SDCardSpiIo	*sdcardio = new SDCardSpiIo();
		sdcardio->CS 	= &top->io_spi_ss;
		sdcardio->CLK 	= &top->io_spi_sclk;
		sdcardio->MOSI 	= &top->io_spi_mosi;
		sdcardio->MISO 	= (CData*)&top->io_spi_miso;
		SDCardSpi *sdcard = new SDCardSpi(sdcardio);
		axiClk->add(sdcard);
		#ifdef TRACE
		//speedFactor = 100e-6;
		//cout << "Simulation caped to " << timeToSec << " of real time"<< endl;
		#endif

		axiClk->add(new VexRiscvTracer(top->Briey_SOTB_Aug2019->axi_core_cpu));

		#ifdef VGA
		Vga *vga = new Vga(top,640,480);
		vgaClk->add(vga);
		#endif

		top->io_coreInterrupt = 0;
	}


	/*bool trigged = false;
	uint32_t frameStartCounter = 0;
	virtual void dump(uint64_t i){
		if(!trigged) {
			if(top->Briey->axi_vgaCtrl->vga_ctrl->io_frameStart) {
				frameStartCounter++;
				if(frameStartCounter < 3*32) cout << "**\n" << endl;
			}
			if(top->Briey->axi_vgaCtrl->vga_ctrl->io_error && frameStartCounter > 3*32) trigged = true;
		}
		if(trigged)Workspace::dump(i);
	}*/


};


struct timespec timer_start(){
    struct timespec start_time;
    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &start_time);
    return start_time;
}

long timer_end(struct timespec start_time){
    struct timespec end_time;
    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &end_time);
    uint64_t diffInNanos = end_time.tv_sec*1e9 + end_time.tv_nsec -  start_time.tv_sec*1e9 - start_time.tv_nsec;
    return diffInNanos;
}



int main(int argc, char **argv, char **env) {

	Verilated::randReset(2);
	Verilated::commandArgs(argc, argv);

	printf("BOOT\n");
	timespec startedAt = timer_start();
    uint32_t gpio_in = 49;
    if (argc > 1)
    {
        printf("Here\n");
        gpio_in = atoi(argv[1]);
    }
    printf("gpio_in %d\n", gpio_in);
	BrieyWorkspace(gpio_in).run(1e9);

	uint64_t duration = timer_end(startedAt);
	cout << endl << "****************************************************************" << endl;


	exit(0);
}
