/**
  ******************************************************************************
  * File Name          : delay.h
  * Description        : ���� �������� ������� �������� �������
  *                      
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 D.Zaur.M
  *
  *
  ******************************************************************************
  */
	
	#include <stdint.h>
	
	/* protoypes	*/
	void delay_init(void);
	void delay_us(uint32_t us);
	void delay_ms(uint32_t ms);
