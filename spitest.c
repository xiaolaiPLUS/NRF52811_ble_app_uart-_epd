#include "nrf_gpio.h"


#include "boards.h"
#include "nrf_delay.h"
#include "nrf_log.h"
#include "nrf_drv_spi.h"
#include "epd42_IO.h"
#include "spitest.h"
#include "nrf_delay.h"
#include "Ap_29demo.h"


static volatile bool spi_xfer_done;  //SPI数据传输完成标志
static const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);  /**< SPI instance. */
uint8_t    spi_tx_buf[256];   /**< TX buffer. */
 uint8_t    spi_rx_buf[256];   /**< RX buffer. */
/**
 * @brief SPI user event handler.
 * @param event
 */
void spi_event_handler(nrf_drv_spi_evt_t const * p_event,
                       void *                    p_context)
{
  spi_xfer_done = true;
}



void spi_init(void)
{
//	nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG(SPI_INSTANCE);
//  spi_config.ss_pin = SPI_CS_PIN;
//  APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, spi_event_handler));	
	//nrf_gpio_cfg_output(SPI_SS_PIN);
	//nrf_gpio_pin_set(SPI_SS_PIN);
	
	  nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
    spi_config.ss_pin   = EPD_CS;
    spi_config.miso_pin = NRF_DRV_SPI_PIN_NOT_USED;
    spi_config.mosi_pin = EPD_MOSI;
    spi_config.sck_pin  = EPD_CLK;
		spi_config.frequency = SPI_FREQUENCY_FREQUENCY_M8;//SPI_FREQUENCY_FREQUENCY_K500; 
    spi_config.irq_priority = 4;   

	
    APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, spi_event_handler, NULL));
	
	
	
	
	

                                              // 在定时器中使用优先级需小于6
}
void SpiAD7799_transfer(uint8_t *p_tx_buffer,uint8_t tx_len,uint8_t *p_rx_buffer,uint8_t rx_len)
{
    //uint8_t len = 1;
	
    
	// Reset rx buffer and transfer done flag
	  spi_xfer_done = false;
	  //APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, spi_tx_buf, len, spi_rx_buf, len));
	//APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, spi_tx_buf, 1, spi_rx_buf, len));
	nrf_drv_spi_transfer(&spi, p_tx_buffer, tx_len, p_rx_buffer, rx_len);
    while (!spi_xfer_done)
    {
        __WFE();
    }

}

void EPD_W21_WriteCMD(unsigned char command)
{

    nrf_gpio_pin_clear(EPD_DC); // command write
	SpiAD7799_transfer((unsigned char[]){command},1,NULL,0);//software reset
	nrf_gpio_pin_set(EPD_DC);
}
void EPD_W21_WriteDATA(unsigned char command)
{

               
	//nrf_gpio_pin_set(EPD_DC);		// command write
	SpiAD7799_transfer((unsigned char[]){command},1,NULL,0);//software reset

}
void EPD_W21_Init(void)
{
	nrf_gpio_pin_clear(EPD_RST);		// Module reset
	nrf_delay_ms(10); //At least 10ms delay 
	nrf_gpio_pin_set(EPD_RST);
	nrf_delay_ms(10);//At least 10ms delay 
	
}

void lcd_chkstatus(void)
{
	printf("lcd_chkstatus\n");
	while(nrf_gpio_pin_read(EPD_BUSY));  //1:BUSY,0:Free 
	nrf_delay_ms(200);                    
	printf("lcd_chkstatus2\n");
}

void PIC_display_Clean(void)
{
    unsigned int i;
		EPD_W21_WriteCMD(0x24);	       //Transfer BW data
	  for(i=0;i<15000;i++)	     
	{
	  EPD_W21_WriteDATA(0xff);
	}
	
		EPD_W21_WriteCMD(0x26);		     //Transfer RED data
	  for(i=0;i<15000;i++)	     
	{
	  EPD_W21_WriteDATA(0x00);
	}
}
void PIC_display(const unsigned char* picData_BW,const unsigned char* picData_R)
{
    unsigned int i;
		EPD_W21_WriteCMD(0x24);	       //Transfer BW data
	  for(i=0;i<15000;i++)	     
	{
	  EPD_W21_WriteDATA(*picData_BW);
	  picData_BW++;
	}
		EPD_W21_WriteCMD(0x26);		     //Transfer RED data
	  for(i=0;i<15000;i++)	     
	{
	  EPD_W21_WriteDATA(~(*picData_R));  
	  picData_R++;
	}

}
void EPD_sleep(void)
{
    EPD_W21_WriteCMD(0X10);  	//deep sleep
		EPD_W21_WriteDATA(0x01);
}
void EPD_refresh(void)
{
		EPD_W21_WriteCMD(0x22);			//DISPLAY REFRESH 	
		EPD_W21_WriteDATA(0xC7);
		EPD_W21_WriteCMD(0x20);
		lcd_chkstatus(); //waiting for the electronic paper IC to release the idle signal
		
}	
void EPD_init(void)
{
	printf("EPD_init.");
		EPD_W21_Init(); //Electronic paper IC reset
printf("EPD_init.11");
  	EPD_W21_WriteCMD(0x12);			//SWRESET
	printf("EPD_init.12");
		lcd_chkstatus(); //waiting for the electronic paper IC to release the idle signal
	
		EPD_W21_WriteCMD(0x74);
		EPD_W21_WriteDATA(0x54);
		EPD_W21_WriteCMD(0x7E);
		EPD_W21_WriteDATA(0x3B);
		EPD_W21_WriteCMD(0x2B);  // Reduce glitch under ACVCOM	
		EPD_W21_WriteDATA(0x04);           
		EPD_W21_WriteDATA(0x63);

		EPD_W21_WriteCMD(0x0C);  // Soft start setting
		EPD_W21_WriteDATA(0x8B);           
		EPD_W21_WriteDATA(0x9C);
		EPD_W21_WriteDATA(0x96);
		EPD_W21_WriteDATA(0x0F);

		EPD_W21_WriteCMD(0x01);  // Set MUX as 300
		EPD_W21_WriteDATA(0x2B);           
		EPD_W21_WriteDATA(0x01);
		EPD_W21_WriteDATA(0x00);     

		EPD_W21_WriteCMD(0x11);  // Data entry mode
		EPD_W21_WriteDATA(0x01);         
		EPD_W21_WriteCMD(0x44); 
		EPD_W21_WriteDATA(0x00); // RAM x address start at 0
		EPD_W21_WriteDATA(0x31); // RAM x address end at 31h(49+1)*8->400
		EPD_W21_WriteCMD(0x45); 
		EPD_W21_WriteDATA(0x2B);   // RAM y address start at 12Bh     
		EPD_W21_WriteDATA(0x01);
		EPD_W21_WriteDATA(0x00); // RAM y address end at 00h     
		EPD_W21_WriteDATA(0x00);
		EPD_W21_WriteCMD(0x3C); // board
		EPD_W21_WriteDATA(0x01); // HIZ

		EPD_W21_WriteCMD(0x18);
		EPD_W21_WriteDATA(0X80);
		EPD_W21_WriteCMD(0x22);
		EPD_W21_WriteDATA(0XB1);	//Load Temperature and waveform setting.
		EPD_W21_WriteCMD(0x20);
		lcd_chkstatus(); //waiting for the electronic paper IC to release the idle signal
		

		EPD_W21_WriteCMD(0x4E); 
		EPD_W21_WriteDATA(0x00);
		EPD_W21_WriteCMD(0x4F); 
		EPD_W21_WriteDATA(0x2B);
		EPD_W21_WriteDATA(0x01);
}


int	epdtest(void)
{
	
	spi_init();
	
	printf("epdtest.");
	//RCC_Configuration();
	//GPIO setting
	//GPIO_Configuration();
	
	//nrf_gpio_pin_clear
	nrf_gpio_cfg_output(EPD_DC);nrf_gpio_pin_set(EPD_DC);
	
	nrf_gpio_cfg_output(EPD_RST);nrf_gpio_pin_set(EPD_RST);
	
	nrf_gpio_cfg_input(EPD_BUSY,NRF_GPIO_PIN_PULLUP);
	
	nrf_gpio_cfg_output(EPD_BS);nrf_gpio_pin_clear(EPD_BS);//4lin spi
	nrf_gpio_cfg_output(EPD_EN);nrf_gpio_pin_set(EPD_EN);
	
	
	
	while(1)	   
	{
    EPD_init(); //EPD init
	  PIC_display(gImage_black1,gImage_red1);//EPD_picture1
		EPD_refresh();//EPD_refresh		
		EPD_sleep();//EPD_sleep,Sleep instruction is necessary, please do not delete!!!
		nrf_delay_ms(1000*80);

		//EPD_Clean
		EPD_init(); //EPD init
	  PIC_display_Clean();//EPD_Clean
		EPD_refresh();//EPD_refresh		
		EPD_sleep();//EPD_sleep,Sleep instruction is necessary, please do not delete!!!
		nrf_delay_ms(1000*80);

	}
}	












