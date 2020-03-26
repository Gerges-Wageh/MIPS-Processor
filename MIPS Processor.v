
module pc(current_pc,next_pc,clk,reset);        // PC 
output reg [31:0] current_pc;
input [31:0] next_pc;
input clk , reset ;
always@(negedge clk or posedge reset)
begin
	if(reset)
	   current_pc<=32'b0000_0000_0000_0000_0000_0000_0000_0000;
	else
	   current_pc<=next_pc;
end
endmodule 

module instruction_memory (instruction,read_instruction,clk);      // INSTRUCYION_MERMORY
output reg [31:0] instruction ;
input [31:0] read_instruction ;
input clk ;
reg [31:0] memory [0:8191] ;
initial
begin

$readmemb("G:\\PyCharm\\Tkinter/Machine_Code.txt",memory);
end
always@(posedge clk)
begin
if((read_instruction>>2)<8191)
instruction<=memory[read_instruction>>2];
end
endmodule

module register_file (read_data_1,read_data_2,read_reg_1,read_reg_2,write_reg,write_data,RegWrite,clk); // Reg_File
input clk ,RegWrite ;
output [31:0] read_data_1 , read_data_2 ;
input [4:0] read_reg_1 , read_reg_2 , write_reg ;
input [31:0] write_data ;
reg [31:0] RF [0:31] ;
integer i;
integer file;
initial
begin
$readmemb("RF.txt",RF);
#500
file = $fopen("RF_OUT.txt");
for(i=0;i<32;i=i+1)
begin
$fwrite(file, "%b // %d \n",RF[i],i);
end
end
assign read_data_1 = RF[read_reg_1];
assign read_data_2 = RF[read_reg_2];
always @(posedge clk)
begin
if(RegWrite)
RF[write_reg] <= write_data ;
end
endmodule

module ALU(zero,result,sh_amt,read_d1,read_d2,aluControl);
output zero;
output reg [31:0] result;
input [4:0]sh_amt;
input [31:0]read_d1,read_d2;
input [3:0]aluControl;
assign zero = ~| result ;

assign z=read_d1 - read_d2;
always @(*)
begin
case(aluControl)
4'b0010:result<=read_d1 + read_d2; //add 
4'b0000:result<=read_d1 & read_d2; //AND
4'b1100:result<= ~(read_d1 | read_d2); //NOR
4'b0001:result<=read_d1 | read_d2; //OR
4'b0110:result<=read_d1 - read_d2;  //SUB
4'b0111:result <=(read_d1<read_d2)? 1:0; //Slt
4'b0011:result<=read_d2 << sh_amt; //sll
endcase
end

endmodule

module ALUControl(ALUop,inst_fn,ALU,Jr_flag);
input [1:0]ALUop;
input [5:0]inst_fn;
output reg[3:0]ALU;
output reg Jr_flag;
always @(ALUop or inst_fn)
begin
Jr_flag=0;
ALU=4'b0000;
case(ALUop)
//Not R-type:
2'b11 :ALU=4'b0001;  //Ori
2'b01 :ALU=4'b0110; //SUB
2'b00 :ALU=4'b0010; //ADD 

//R-Format : 
2'b10 : begin
 case(inst_fn)
   6'b100000:ALU=4'b0010 ; //Add
   6'b100100:ALU=4'b0000; //And
   6'b001000:Jr_flag=1'b1;//Jr 
   6'b100111:ALU=4'b1100; //NOR
   6'b101010:ALU=4'b0111; //Slt
   6'b000000:ALU=4'b0011; //Sll  
   6'b100010:ALU=4'b0110; //Sub 
   6'b100101:ALU=4'b0001; //Or
	default :ALU=4'bxxxx;
endcase
end
default: ALU=4'bxxxx; //useless 
endcase
end
endmodule


module control_unit(opcode,reset,reg_dst,mem_to_reg,alu_op,jump,branch,mem_read,mem_write,alu_src,reg_write,bne);  
input reset;
input[5:0] opcode;
output reg[1:0] alu_op , mem_to_reg , reg_dst;
output reg jump,branch,mem_read,mem_write,alu_src,reg_write,bne;
always@(*)   
begin  
if(reset==1'b1)
begin  
reg_dst = 2'b00;  
 mem_to_reg = 2'b00;  
alu_op = 2'b00;  
jump = 1'b0;  
branch = 1'b0;  
bne = 1'b0;
mem_read = 1'b0;  
mem_write = 1'b0; 
alu_src = 1'b0;  
reg_write = 1'b0;
end 
else
begin  
case(opcode)   
 6'b000000: begin //R-Format(and-or-add-slt-jr-sll)
reg_dst = 2'b01;  
mem_to_reg = 2'b00;  
alu_op = 2'b10;  
jump = 1'b0;  
branch = 1'b0; 
bne = 1'b0;
mem_read = 1'b0;  
mem_write = 1'b0;  
alu_src = 1'b0;  
reg_write = 1'b1; 
end  

  6'b001101: begin // Ori
reg_dst = 2'b00;  
mem_to_reg = 2'b00;  
alu_op = 2'b11;  
jump = 1'b0;  
branch = 1'b0;
bne = 1'b0;
mem_read = 1'b0;  
mem_write = 1'b0;  
alu_src = 1'b1;  
reg_write = 1'b1;  
  end  
 
    6'b000010: begin // j  
reg_dst = 2'b00;  
mem_to_reg = 2'b00;  
alu_op = 2'b00;  
jump = 1'b1;  
branch = 1'b0; 
bne = 1'b0;
mem_read = 1'b0;  
mem_write = 1'b0;  
alu_src = 1'b0;  
reg_write = 1'b0;  
   end  
     6'b000011: begin // jal  
reg_dst = 2'b10;  
mem_to_reg = 2'b10;  
alu_op = 2'b00; 
jump = 1'b1;  
branch = 1'b0; 
bne = 1'b0; 
mem_read = 1'b0;  
mem_write = 1'b0;  
alu_src = 1'b0;  
reg_write = 1'b1; 
    end  
     6'b100011: begin // lw  
reg_dst = 2'b00;  
mem_to_reg = 2'b01;  
alu_op = 2'b00;  
jump = 1'b0;  
branch = 1'b0;  
bne = 1'b0;
mem_read = 1'b1;  
mem_write = 1'b0;  
alu_src = 1'b1;  
reg_write = 1'b1;  
     end  
      6'b101011: begin // sw  
reg_dst = 2'b00;  
mem_to_reg = 2'b00;  
alu_op = 2'b00;  
jump = 1'b0;  
branch = 1'b0; 
bne = 1'b0; 
mem_read = 1'b0;  
mem_write = 1'b1;  
alu_src = 1'b1;  
reg_write = 1'b0;  
     end  
      6'b000100: begin // beq  
reg_dst = 2'b00;  
mem_to_reg = 2'b00;
alu_op = 2'b01;  
jump = 1'b0; 
branch = 1'b1; 
bne = 1'b0; 
mem_read = 1'b0;  
mem_write = 1'b0;  
alu_src = 1'b0;  
reg_write = 1'b0;  
     end  
      6'b001000: begin // addi  
reg_dst = 2'b00;  
mem_to_reg = 2'b00;  
alu_op = 2'b00;  
jump = 1'b0;  
branch = 1'b0; 
bne = 1'b0; 
mem_read = 1'b0;  
mem_write = 1'b0;  
alu_src = 1'b1;  
reg_write = 1'b1;  
     end  
      6'b000101: begin
reg_dst = 2'b00;  
mem_to_reg = 2'b00;
alu_op = 2'b01;  
jump = 1'b0; 
branch = 1'b0; 
bne = 1'b1; 
mem_read = 1'b0;  
mem_write = 1'b0;  
alu_src = 1'b0;  
reg_write = 1'b0; 
      end
 default: begin  
reg_dst = 2'bxx;  
mem_to_reg = 2'bxx;  
alu_op = 2'bxx;  
jump = 1'bx;  
branch = 1'bx;  
bne = 1'bx;
mem_read = 1'b0;  
mem_write = 1'b0;  
alu_src = 1'bx;  
reg_write = 1'b0;  
    end  
endcase  
      end  
 end  
 endmodule 


module Data_Memory(read_data , address , write_data , MemWrite , MemRead,clk);
output [31:0] read_data ;
input [31:0] address , write_data;
input MemWrite , MemRead , clk ;
reg [31:0] D_M [0:8191] ;
integer i ;
integer file ;
initial 
begin
$readmemb("Memory.txt",D_M);
#500
file = $fopen("DM_OUT.txt");
for(i=0;i<8192;i=i+1)
begin
$fwrite(file, "%b // %d \n",D_M[i],i);
end
end
always@(posedge clk)
begin
if(MemWrite)
D_M[(address>>2)] <= write_data ;
end
assign read_data =(MemRead==1'b1) ? D_M[(address>>2)] : 16'b0 ;
endmodule  


module adder (out , inp1 , inp2); // adder 
output  [31:0] out ;
input [31:0] inp1 , inp2 ;
assign out = inp1 + inp2 ;
endmodule 



module pc_adder(op ,ip );
output [31:0] op ;
input [31:0] ip ;
parameter  const = 32'b_0000_0000_0000_0000_0000_0000_0000_0100 ;
assign op = ip + const ; 
endmodule


module sign_extend(in,out);
input [15:0]in;
output [31:0]out;
assign out[15:0] = in[15:0];
assign out[31:16]={16{in[15]}};
endmodule



module mux3_ip_32(z,x,y,w,select);
output z [31:0] ;
reg [31:0] z ;
input [31:0] x , y ,w;
input [1:0] select ;
always @(x or y or select)
begin
if(select==2'b00)
    z=x;
  else if(select==2'b01)
    z=y;
   else 
     z=w ;
end
endmodule

module mux2_ip_32(z,x,y,select);
output z [31:0] ;
reg [31:0] z ;
input [31:0] x , y ;

input select ;
always @(x or y or select)
begin
if(select==1'b0)
    z=x;
  else
    z=y;
end
endmodule


module mux3_ip_5(z,x,y,w,select);
output z [4:0] ;
reg [4:0] z ;
input [4:0] x , y , w ;
input [1:0] select ;
always @(x or y or select)
begin
if(select==2'b00)
    z=x;
  else if(select==2'b01)
    z=y;
     else 
	z=w;
end 
endmodule

module Top_Module(reset ,clk);
input clk , reset ;
wire [31:0] current_pc,instruction,read1,read2,write_data,sign_extend_op,z,result,read_data,pc_adder_op,adder_op,zero_dest,new_zero_dest,
jump_wire,next_pc,PC_in;

 
wire [4:0] first_mux_op ;
wire [1:0] alu_op , mem_to_reg , reg_dst;
wire jump,branch,mem_read,mem_write,alu_src,reg_write,Jr_flag,write_enable,zero,sel,sel2,bne;
wire [3:0] ALU ;
assign jump_wire = {pc_adder_op[31:28],instruction[25:0],2'b0};
assign sel = branch & zero;
assign sel2 =bne & (!zero);
assign write_enable=Jr_flag ^ reg_write;
pc my_pc (current_pc,PC_in,clk,reset);
instruction_memory my_mem(instruction,current_pc,clk); 
mux3_ip_5 My_first_mux3 (first_mux_op,instruction[20:16],instruction[15:11],5'b11111,reg_dst);
register_file my_reg_file(read1,read2,instruction[25:21],instruction[20:16],first_mux_op,write_data,write_enable,clk);
sign_extend my_SE(instruction[15:0],sign_extend_op);
ALUControl my_ALU_Control (alu_op,instruction[5:0],ALU,Jr_flag);
control_unit my_control_unit(instruction[31:26],reset,reg_dst,mem_to_reg,alu_op,jump,branch,mem_read,mem_write,alu_src,reg_write,bne);  
mux2_ip_32 my_first_mux2(z,read2,sign_extend_op,alu_src);
ALU My_ALU(zero,result,instruction[10:6],read1,z,ALU);
Data_Memory my_DM(read_data,result,read2,mem_write,mem_read,clk);
mux3_ip_32 my_second_Mux32(write_data,result,read_data,(current_pc+4),mem_to_reg);
pc_adder my_pc_adder(pc_adder_op ,current_pc );
adder my_adder (adder_op,(sign_extend_op<<2) , pc_adder_op); 
mux2_ip_32 My_mux11(zero_dest,pc_adder_op,adder_op,sel); // first mux after adder 
mux2_ip_32 My_Mux111(new_zero_dest,zero_dest,adder_op,sel2);
mux2_ip_32 My_Mux1111(next_pc,new_zero_dest,jump_wire,jump); // last mux ... sheel zero dest w 7ot el op bta3 el mux el gded


mux2_ip_32 My_JR_MUX (PC_in,next_pc,read1,Jr_flag);
endmodule 

module projectdone();
reg clk ;
reg reset ;
Top_Module my_mips (reset,clk);
initial
begin  
clk = 0;  
forever #10 clk = ~clk;  
end  
initial
begin    
reset = 1;  
#5
reset = 0;            
end 
endmodule 











