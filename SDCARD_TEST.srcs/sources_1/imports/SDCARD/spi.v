`timescale 1ns/1ps
		       //HI_FREQ_DIV=clk division for the high freq,LO_FREQ_DIV=clk division for low freq [can be any even number >=2]
module spi #(parameter HI_FREQ_DIV=12,LO_FREQ_DIV=30,SPI_MODE=0) (  //SPI_MODE can be either 0,1,2,3
	input wire clk,rst,                            
	//SPI control
	input clk_div, //1: high frequency , 0=low frequency
	input wire rd,wr,hold,  //pins to start read or write operation, hold is for holding the clock for multibyte read/write 
	input wire[7:0] wr_data, //data to be sent to the slave
	output wire[7:0] rd_data, //data received from the slave
	output reg done_tick, //ticks if either write or read operation is finished
	output ready, //can perform read/write operation only if ready is "1" (except for multibyte read/write where state will never go to IDLE)
	//SPI pinouts
	input wire miso,
	output wire mosi,
	output wire sclk,cs_n
	);
	
	/* template
	    spi #(.HI_FREQ_DIV(12),.LO_FREQ_DIV(30),.SPI_MODE(0)) m0 //High freq: 12MHz/12=1MHz , Low freq: 12MHz/30=400KHz
    ( 
        .clk(clk),
        .rst(rst),                                
        //SPI control
        .rd(rd),
        .wr(wr),
        .hold(hold),  //pins to start read or write operation, hold is for holding the clock for multibyte read/write 
        .wr_data(wr_data), //data to be sent to the slave
        .rd_data(rd_data), //data received from the slave
        .done_tick(done_tick), //ticks if either write or read operation is finished
        .ready(ready), //can perform read/write operation only if ready is "1" (except for multibyte read/write where state will never go to IDLE)
        //SPI pinouts
        .miso(miso),
        .mosi(mosi),
        .sclk(sclk),
        .cs_n(cs_n_1)
	);
	*/
	
	localparam IDLE=0,
               WRITE=1,
               WRITE_DELAY=2,
               READ=3,
               HOLD=4;
		   
		   
	localparam CPOL=SPI_MODE[1], //clock polarity
		       CPHA=SPI_MODE[0]; //clock phase


	reg[2:0] state_q=0,state_d; 
	reg[3+!CPHA:0] count_q=0,count_d; //counter for shifting 8 times
	reg[7:0] wr_data_q=0,wr_data_d; //contains data to be sent to the slave
	reg[7:0] rd_data_q=0,rd_data_d;  //contains data received from the slave
	reg[$clog2(LO_FREQ_DIV+1)-1:0] clk_ctr_q=0,clk_ctr_d; //counter for clock division
    reg cs_n_q=0,cs_n_d;
    reg sclk_q=0,sclk_d;
    reg hold_q=0,hold_d;
    wire[$clog2(LO_FREQ_DIV+1)-1:0] CLK_DIV=clk_div? HI_FREQ_DIV:LO_FREQ_DIV;    
    
	//register operations
	always @(posedge clk,posedge rst) begin
		if(rst) begin
			state_q<=0;
			count_q<=0;
			wr_data_q<=0;
			rd_data_q<=0;
			clk_ctr_q<=0;
			cs_n_q<=0;
			sclk_q<=0;
			hold_q<=0;
		end
		else begin
			state_q<=state_d;
			count_q<=count_d;
			wr_data_q<=wr_data_d;
			rd_data_q<=rd_data_d;
			clk_ctr_q<=clk_ctr_d;
			cs_n_q<=cs_n_d;
			sclk_q<=sclk_d;
			hold_q<=hold_d;
		end
		
	end
	
	//FSM logic
	always @* begin
		state_d=state_q;
		count_d=count_q;
		wr_data_d=wr_data_q;
		rd_data_d=rd_data_q;
		clk_ctr_d=clk_ctr_q;
		cs_n_d=cs_n_q;
		sclk_d=sclk_q;
		hold_d=hold_q;
        done_tick=0;
        
		//logic for clk division and sclk 
		clk_ctr_d=(state_q==IDLE || state_q==HOLD || clk_ctr_q==CLK_DIV-1)? 0:clk_ctr_q+1'b1;
		sclk_d=(clk_ctr_q==CLK_DIV-1) || (clk_ctr_q==((CLK_DIV>>1)-1))? !sclk_q:sclk_q; 

		case(state_q) 
			IDLE: begin
                    sclk_d=CPOL; //CPOL (clock polarity) is the initial value of clock 
                    cs_n_d=1; //cs_n is active low
                    count_d=0;
                    if(wr) begin
                        cs_n_d=0; 
                        wr_data_d=wr_data;			
                        state_d=CPHA? WRITE_DELAY:WRITE; //CPHA=1: MOSI is delayed by half-clk cycle
                    end
                    if(rd) begin
                        cs_n_d=0;
                        state_d=READ;
                     end
			      end
			      
			WRITE: if(clk_ctr_q==(CLK_DIV>>CPHA)-1) begin //CPHA=1: MOSI is updated at half-clk cycle , CPHA=0: MOSI is updated at end-of-clk cycle
					wr_data_d=wr_data_q<<1;//shift data 8 times (MSB first)
					count_d=(count_q==7)? 0:count_q+1'b1; 
                    if(count_q==7) begin
                        done_tick=1;
                        if(wr) begin //write again (multibyte write)
                        count_d=0;
                        wr_data_d=wr_data;			
                        state_d=WRITE;
                        end
                        else if(rd) begin //read after writing
                            count_d=0;
                            state_d=READ; 
                        end
                        else if(hold) begin
                            sclk_d=CPOL;
                            state_d=HOLD;
                        end
                        else begin //go back to IDLE
                            state_d=IDLE;
                            cs_n_d=1;
                            sclk_d=CPOL;
                        end
                        
                     end
				    end
				    
	 WRITE_DELAY: if(clk_ctr_q==(CLK_DIV>>1)-1) state_d=WRITE; //for CPHA=1 when MOSI is delayed by half-clk cycle
	 
			READ:if(clk_ctr_q==(CLK_DIV>>!CPHA)-1)begin //CPHA=1: MISO is read at end-of-clk cycle , CPHA=0: MISO is read at half-clk cycle
					rd_data_d={rd_data_q[6:0],miso}; //shift read data (MSB first)
					count_d=(count_q==7+!CPHA)? 0:count_q+1'b1; //CPHA=0 needs one more clk cycle(thus 0-to-8) BEFORE GOING IDLE (not needed if not going to idle after reading)
					if(count_q==7) begin 
                        done_tick=1;
                        hold_d=hold;
                        if(wr) begin //write after reading
                            count_d=0;
                            wr_data_d=wr_data;			
                            state_d=WRITE;
                        end
                        else if(rd) begin
                            count_d=0;
                            state_d=READ; //multibyte read
                        end    
                        else if(CPHA) begin //for CPHA=1 only,goes IDLE
                            state_d=IDLE;
                            sclk_d=CPOL;
                            cs_n_d=1;
                            if(hold) begin
                                state_d=HOLD;
                                cs_n_d=0;
                            end
                        end
                        
                     end  
                    if(count_q==8) begin //for CPHA=0 only, goes IDLE
                            rd_data_d=rd_data_q; //we added one more clk cycle for CPHA=0 before going IDLE but rd_data_q must not be updated again for that last clk cycle
                            state_d=IDLE;
                            sclk_d=CPOL;
                            cs_n_d=1;
                            if(hold_q) begin
                                state_d=HOLD;
                                cs_n_d=0;
                            end
                     end
				    end
				    
		        HOLD: begin
		                sclk_d=CPOL; //CPOL (clock polarity) is the initial value of clock 
                        count_d=0;
                        hold_d=0;
                        if(wr) begin
                            wr_data_d=wr_data;			
                            state_d=CPHA? WRITE_DELAY:WRITE;
                        end
                        if(rd) state_d=READ;
                      end
		     default: state_d=IDLE;
		endcase
	end
	assign mosi=(state_q==WRITE)? wr_data_q[7]:1'b1; //asserted to "1" when not sending/receiving data
	assign rd_data=rd_data_d; 
	assign sclk=sclk_q;
	assign cs_n=cs_n_q;
    assign ready=state_q==IDLE || state_q==HOLD;
endmodule	
