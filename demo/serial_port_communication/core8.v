module core8(
    input clk,
    input n_rst,
    // �����źţ���ֹ�Ż���
    output r00,
    output r01,
    output r02,
    output r03,
    output r04,
    output r05,
    output r06,
    output r07,
    // ����ͨ�Žӿ�
    input rx, // ���ڽ���
    output tx // ���ڷ���
);
    wire rst = ~n_rst; // �͵�ƽ��λ�ź�

    wire [7:0] databus;
    
    wire alua_in;
    wire alub_in;
    wire alu_out;
    wire[2:0] sel;
    wire cf;
    wire zf;
    wire pc_inc; // �������������
    wire pc_jump; // �����������ת
    wire pc_out; // ������������������
    wire pc_reset; // �����������λ
    wire[2:0] s_addrbus; // Դ�Ĵ�����ַ����
    wire[2:0] t_addrbus; // Ŀ��Ĵ�����ַ���� 
    wire iMDRout;
    wire dmdr_iin; // ���ݴ洢������
    wire dmdr_iout; // ���ݴ洢�������
    wire ir_in; // ָ��Ĵ�������
    wire imm_out; // ���������
    wire[3:0] opcode; // ������
    wire[3:0] operand; // ������
    wire[7:0] cnt;
    wire[7:0] r3dbg;
    
    
    // ��������������
    wire [7:0] alu_bus;
    wire [7:0] pc_bus;
    wire [7:0] ir_bus;
    wire [7:0] reg_bus[0:3];
    wire cur_reg_bus_in; // ���Ƶ�Ԫ�Ĵ��������
    wire cur_reg_en_in; // ���Ƶ�Ԫ�Ĵ�����ʹ���ź�
    wire cur_reg_bus_out; // ��ǰ�Ĵ��������
    wire cur_reg_en_out; // ��ǰ�Ĵ�����ʹ���ź�
    wire [7:0] imem_bus;
    wire [7:0] dmem_bus;
    reg [3:0] cur_send_sta = 4'b0000; // ��ǰ����״̬�Ĵ���
    wire mem_as_target; // ���ݴ洢����ΪĿ���ַ����
    wire mem_as_source;

    assign r00 = r3dbg[0];
    assign r01 = r3dbg[1];
    assign r02 = r3dbg[2];
    assign r03 = r3dbg[3];
    assign r04 = r3dbg[4];
    assign r05 = r3dbg[5];
    assign r06 = r3dbg[6];
    assign r07 = r3dbg[7];

    // reg clk2 = 1;
    // reg clk4 = 1;
    // always @(posedge clk) clk2 <= ~clk2;
    // always @(posedge clk2) clk4 <= ~clk4;

    // ��databus��·����
    assign databus = 
        (pc_out) ? pc_bus :          // ������������
        (iMDRout) ? imem_bus :      // ָ��洢�����
        (dmdr_iout) ? dmem_bus :    // ���ݴ洢�����
        (imm_out) ? ir_bus :          // ָ��Ĵ�������
        (alu_out) ? alu_bus :       // ALU���
        s_addrbus[2] == 1'b0 ? reg_bus[{s_addrbus[1:0]}] : // Դ�Ĵ��������
        8'bzzzz_zzzz;                    // �Ĵ��������

    // Instantiate ALU
    ALU alu_inst(
        .databus(alu_bus),
        .inbus(databus),
        .clk(clk),
        .alua_in(alua_in),
        .alub_in(alub_in),
        .sel(sel),
        .cf(cf),
        .zf(zf) // ���־��ZF��
    );


    //----------------------------------
    // ��ģ��ʵ����
    //----------------------------------
    // ���������
    PC u_PC (
        .clk(clk),
        .pc_inc(pc_inc),
        .pc_jump(pc_jump),
        .databus(pc_bus),
        .inbus(databus), // �����������ߣ�����ALU������ģ�飩
        .reset(pc_reset | rst)
    );
    // ͨ�üĴ���
    reg_file u_reg_file (
        .clk(clk),
        .rst(rst | pc_reset), // ȫ�ָ�λ�ź�
        .t_addrbus(t_addrbus),
        .s_addrbus(s_addrbus),
        .databus0(reg_bus[0]),
        .databus1(reg_bus[1]),
        .databus2(reg_bus[2]),
        .databus3(reg_bus[3]),
        .inbus(databus),
        .cur_reg_bus_in(cur_reg_bus_in), // ���Ƶ�Ԫ�Ĵ��������
        .cur_reg_en_in(cur_reg_en_in), // ���Ƶ�Ԫ�Ĵ�����ʹ���ź� 
        .cur_reg_bus_out(cur_reg_bus_out), // ��ǰ�Ĵ��������
        .cur_reg_en_out(cur_reg_en_out), // ��ǰ�Ĵ�����ʹ���ź�
        .r3dbg(r3dbg)
    );

    // ָ��洢��
    inst_mem u_inst_mem (
        .clk(clk),
        .pc_addr(databus),      // �����߻�ȡ��ַ
        .databus(imem_bus)      // ���������
    );

    // ���ݴ洢��
    DataMemory u_data_memory (
        .clk(clk),
        // .rst(rst),
        .mem_as_target(mem_as_target), // ���ݴ洢����ΪĿ���ַ����
        .mem_as_source(mem_as_source),
        .databus(dmem_bus),      // ��������
        .inbus(databus), // �����������ߣ�����ALU������ģ�飩
        .dmdr_iin(dmdr_iin),    // ����ʹ��
        .cnt(cnt) // ���������������UART���ͣ�
    );

    // ָ��Ĵ���
    IR u_IR (
        .clk(clk),
        .databus(ir_bus),
        .inbus(databus), // �����������ߣ�����ALU������ģ�飩
        .ir_in(ir_in),
        .opcode(opcode),              // �ڲ�������CU
        .operand(operand)              // �ڲ�������CU
    );

        // ���Ƶ�Ԫ
    CU u_CU (
        .clk(clk),
        .rst(rst),         // ȫ�ָ�λ�ź�
        .opcode(opcode),   // ֱ������IR�ڲ��ź�
        .operand(operand),
        .pc_reset(pc_reset),       // ���ø�λ
        .pc_out(pc_out),
        .pc_inc(pc_inc),
        .pc_jump(pc_jump), // PC��ת
        .iMDRout(iMDRout),
        .ALUAin(alua_in),
        .ALUBin(alub_in),
        .ALUout(alu_out),
        .ALUop(sel),
        .cf(cf),
        .zf(zf), // ���־��ZF��
        .dmdr_iin(dmdr_iin),
        .dmdr_iout(dmdr_iout),
        .mem_as_target(mem_as_target), // ���ݴ洢����ΪĿ���ַ����
        .mem_as_source(mem_as_source),
        .ir_in(ir_in),
        .imm_out(imm_out),
        .s_addrbus(s_addrbus),
        .t_addrbus(t_addrbus),
        .cnt(cnt),
        .rx(rx), // ���ڽ���
        .tx(tx), // ���ڷ���
        .reg_bus_in(cur_reg_bus_in),  // �Ĵ��������
        .reg_en_in(cur_reg_en_in),    // �Ĵ�����ʹ���ź�
        .reg_bus_out(cur_reg_bus_out), // ��ǰ�Ĵ��������
        .reg_en_out(cur_reg_en_out)  // ��ǰ�Ĵ�����ʹ���ź�
    );

endmodule

//PC
module PC(
    input         clk,
    input         pc_inc,       // PC�Ƿ�����
    input         pc_jump,      // PC�Ƿ���ת
    input         reset,
    input      [7:0] inbus,        // ������������
    output reg [7:0] databus       // ͳһ��������
);
    // reg [7:0] cur_pc;     // ��ǰ���������ֵ

    //PC�����Ĵ���
    wire [7:0] next_pc;
    // ��̬������������ȫ���ⲿ���ƣ�
    Incrementer PCInc(
        .op(databus),
        .next_op(next_pc)
    );
    // ���������
    // assign databus = cur_pc; // ��pc_outΪ1ʱ�������ǰPCֵ������Ϊ����̬
    // PC�����߼�
    always @(posedge clk or posedge reset) begin
        if (reset)              // PC����
            databus <= 8'h00;
        else if (pc_inc)        // PC����
            databus <= next_pc;
        else if (pc_jump) // PC��ת
            databus <= inbus;  // ��ת��ָ����ַ
        else databus <= databus;  // PC���䣨���źţ�
    end
endmodule

//ALU
module ALU(databus,inbus,clk,alua_in,alub_in,sel,cf,zf);
	output reg [7:0] databus;       // ͳһ��������
    input [7:0] inbus;
	input clk;
	input alua_in;
	input alub_in;
	input[2:0] sel;
	output cf, zf;
	
	wire[7:0] op1;
	wire[7:0] op2;
	wire [7:0] result;

	
	OPReg A(clk,alua_in,inbus,op1);
    OPReg B(clk, alub_in,inbus,op2);
	ALUCore u1(op1,op2,sel,result,cf,zf);
	// OutputBufReg R(clk,result,alu_out,databus);
    always @(*) begin

			databus<=result;
	end

endmodule

module OPReg(clk,we,din,dout);
	input clk;
	input we;
	input[7:0] din;
	output[7:0] dout;
	reg[7:0] dout;
	
	always @(posedge clk)
	begin
		if(we==1'b1)
		begin
			dout <= din;
		end
	end
endmodule

module ALUCore(
    input [7:0]  op1,
    input [7:0]  op2,
    input [2:0]  sel,    // ��չΪ3λ������
    output reg [7:0] result,
    output reg       cf,
    output reg       zf // ���־��ZF��
);
    
    // �ӷ����ӿ�
    reg [7:0] adder_op1;
    reg [7:0] adder_op2;
    reg suben;
    wire [7:0] adder_sum;
    wire adder_cout;
    
    Adder u1(adder_op2, adder_op1, suben, adder_sum, adder_cout);
    
    always @(*) begin
        case(sel)
            // ��������
            3'b000, 3'b001: begin  // �ӷ�/����
                // ��������ʵ����Դ��Ŀ�꣬���Է�������
                adder_op1 = op1;
                adder_op2 = op2;
                suben = sel[0];
                result = adder_sum;
                cf = adder_cout;
            end
            3'b010: begin  // ����
                adder_op1 = 8'b0;
                adder_op2 = 8'b0;
                suben = 1'b0; // ���Ʋ���Ҫ����
                cf = 1'b0; // ���Ʋ�������λ
                // if(adder_op1[7] | adder_op1[6] | adder_op1[5] | adder_op1[4] | adder_op1[3]) begin
                //     result = 8'b0; // ���ƺ�ȫ��
                // end else begin
                result = op2 << op1;
                // end
            end
            3'b011: begin  // ����
                adder_op1 = 8'b0;
                adder_op2 = 8'b0;
                suben = 1'b0; // ���Ʋ���Ҫ����
                cf = 1'b0; // ���Ʋ�������λ
                // if(adder_op1[7] | adder_op1[6] | adder_op1[5] | adder_op1[4] | adder_op1[3]) begin
                //     result = 8'b0; // ���ƺ�ȫ��
                // end else begin
                result = op2 >> op1;
                // end
            end
            // �߼�����
            3'b100: begin  // �루AND��
				adder_op1 = 8'b0;
                adder_op2 = 8'b0;
                suben = 1'b0;
                result = op1 & op2;
                cf = 1'b0;  // �߼���������CF
            end
            3'b101: begin  // ��OR��
				adder_op1 = 8'b0;
                adder_op2 = 8'b0;
                suben = 1'b0;
                result = op1 | op2;
                cf = 1'b0;
            end
            3'b110: begin  // �ǣ�NOT��
				adder_op1 = 8'b0;
                adder_op2 = 8'b0;
                suben = 1'b0;
                result = op2 ^ 8'hFF; // ȡ��
                cf = 1'b0;
            end
            3'b111: begin  // ���XOR��
				adder_op1 = 8'b0;
                adder_op2 = 8'b0;
                suben = 1'b0;
                result = op1 ^ op2;
                cf = 1'b0;
            end
        endcase
        zf = (result == 8'b0); // ���־��ZF��Ϊ1��ʾ���Ϊ0
    end
endmodule


module Adder(
    input [7:0]  op1,     // ������1
    input [7:0]  op2,     // ������2
    input        suben,   // ����ʹ�ܣ�1=������
    output [7:0] sum,     // ��/����
    output       co       // ��λ/��λ
);
    // ����ת��Ϊ����ӷ�
    wire [7:0] adj_op2 = op2 ^ {8{suben}};
    wire cin = suben;     // ����ʱcin=1������+1��

    // ����λ��Generate���ʹ���λ��Propagate��
    wire [7:0] G = op1 & adj_op2;
    wire [7:0] P = op1 ^ adj_op2;

    // ��ǰ��λ���㣨�����Ż���
    wire [7:0] C;
    assign C[0] = cin;
    assign C[1] = G[0] | (P[0] & C[0]);
    assign C[2] = G[1] | (P[1] & G[0]) | (P[1] & P[0] & C[0]);
    assign C[3] = G[2] | (P[2] & G[1]) | (P[2] & P[1] & G[0]) | (P[2] & P[1] & P[0] & C[0]);
    
    // 4λ����λ���ؼ��Ż��㣩
    wire C4 = G[3] | (P[3] & G[2]) | (P[3] & P[2] & G[1]) | (P[3] & P[2] & P[1] & G[0]) | 
              (P[3] & P[2] & P[1] & P[0] & C[0]);
    
    assign C[4] = C4;
    assign C[5] = G[4] | (P[4] & C4);
    assign C[6] = G[5] | (P[5] & G[4]) | (P[5] & P[4] & C4);
    assign C[7] = G[6] | (P[6] & G[5]) | (P[6] & P[5] & G[4]) | (P[6] & P[5] & P[4] & C4);

    // ���ս�λ�ͽ��
    assign co = suben ^ (G[7] | (P[7] & G[6]) | (P[7] & P[6] & G[5]) | 
                (P[7] & P[6] & P[5] & G[4]) | (P[7] & P[6] & P[5] & P[4] & C4));
    assign sum = P ^ C;
endmodule

//Register File
module reg_file(
    input         clk,
    input       rst,
    // ˫��ַ���߽ӿ�
    input  [2:0]  t_addrbus,    // Ŀ���ַ����
	input  [2:0]  s_addrbus,
    
    output reg [7:0]  databus0, // ˫���������ߣ�dMDR��Ĵ�����֮�佻���ߣ�
    output reg [7:0]  databus1, // ˫���������ߣ�dMDR��Ĵ�����֮�佻���ߣ�
    output reg [7:0]  databus2, // ˫���������ߣ�dMDR��Ĵ�����֮�佻���ߣ�
    output reg [7:0]  databus3, // ˫���������ߣ�dMDR��Ĵ�����֮�佻���ߣ�
    input [7:0]  inbus,  // �����������ߣ�����ALU������ģ�飩
    // �����ź�
    input cur_reg_bus_in, // ���Ƶ�Ԫ�Ĵ��������
    input cur_reg_en_in, // ���Ƶ�Ԫ�Ĵ�����ʹ���ź�
    output reg cur_reg_bus_out, // ��ǰ�Ĵ��������
    input cur_reg_en_out, // ��ǰ�Ĵ�����ʹ���ź�
    output [7:0] r3dbg
);
    assign r3dbg = databus3;
    
    //------------------------------------------
    // ͬ��д�߼�
    //------------------------------------------
    always @(negedge clk) begin
        // Ŀ��Ĵ���д�루��ַ��Чʱ��
        if (cur_reg_en_in) begin
            case (t_addrbus[1:0]) // ��2λѡ��Ĵ���
                2'd0: databus0[7] <= cur_reg_bus_in; // R0
                2'd1: databus1[7] <= cur_reg_bus_in; // R1
                2'd2: databus2[7] <= cur_reg_bus_in; // R2
                2'd3: databus3[7] <= cur_reg_bus_in; // R3
            endcase
        end
        else if (cur_reg_en_out) begin
            case(s_addrbus[1:0]) // ��2λѡ��Ĵ���
                2'd0: cur_reg_bus_out <= databus0[0]; // R0
                2'd1: cur_reg_bus_out <= databus1[0]; // R1
                2'd2: cur_reg_bus_out <= databus2[0]; // R2
                2'd3: cur_reg_bus_out <= databus3[0]; // R3
            endcase
        end
        else if (t_addrbus[2] == 1'b0) begin  // ���λ=0ʱ��Ч
            case (t_addrbus[1:0]) // ��2λѡ��Ĵ���
                2'd0: databus0 <= inbus; // R0
                2'd1: databus1 <= inbus; // R1
                2'd2: databus2 <= inbus; // R2
                2'd3: databus3 <= inbus; // R3
            endcase
        end

        if (rst) begin
            // ȫ���Ĵ�����λΪ0
            databus0 <= 8'b0;
            databus1 <= 8'b0;
            databus2 <= 8'b0;
            databus3 <= 8'b0;
        end
    end
endmodule

//ָ��洢��
module inst_mem(
    input         clk,
    input  [7:0]  pc_addr,       // �����������ַ����
    output  reg [7:0]  databus       // �����������
);
    // �洢������ - ��ȷΪROM
    (* romstyle = "M9K" *) reg [7:0] memory [0:255];

	initial begin
		$readmemb("uart_ic.hex", memory); // ���ǳ�ʼ��
	end
    
    // ͬ����ȡ
    always @(posedge clk) begin
        databus <= memory[pc_addr]; // ���������
    end
    // wire [7:0] dataout;
    // wire dv;
    // reg read_en = 0;

    // ufm u_ufm_inst(
    //     .addr(pc_addr),
    //     .nread(~read_en),
    //     .data_valid(dv),
    //     .dataout(dataout)
    // );

    // always @(posedge clk) begin
    //     if (dv)
    //         databus <= dataout;
    // end

    // always @(posedge clk_2) begin
    //     read_en <= ~read_en;
    // end

endmodule


module DataMemory(
    input         clk,
    // input         rst,
    // ˫��ַ���߽ӿ�
    input  mem_as_source,
    // input  [2:0]  t_addrbus,    // Ŀ���ַ����
    input  mem_as_target,
    output reg  [7:0]  databus,      // ˫���������ߣ�dMDR����������֮�佻���Ĵ�����
    input [7:0] inbus, // ���뵽dMDR���������ߣ�����ALU������ģ�飩
    // �����ź�
    input       dmdr_iin,     // dMDR����������ʹ��
    input [7:0] cnt // ���������������UART���ͣ� - ֱ�����ӵ�DataMemory
);

    (* ramstyle = "no_rw_check, logic" *)  // ���ö�д��ͻ���
    reg [7:0] memory [0:7];


    // DataMemoryʵ��
    wire [7:0] mem_dout;
    wire        mem_we;
    wire [7:0] mem_addr; // �ڴ��ַ�Ĵ���

    // дʹ���߼���t_addrbus==100ʱд��
    // wire mem_as_target = (t_addrbus == 3'b100);
    // wire mem_as_source = (s_addrbus == 3'b100);

    // assign mem_addr = inbus; // ��ַ�Ĵ�������
    // assign mem_din = mem_as_target ? inbus : 8'bzzzz_zzzz; // д���������ߣ�����̬��

    // MDR���ڴ桢���ߵ�����������
    always @(negedge clk) begin
        // �������뵽MDR
        if (dmdr_iin)
            databus <= inbus;
        // MDRд���ڴ�
        else if (mem_as_target)
            memory[mem_addr[2:0]] <= databus;
        // �ڴ����MDR
        else if (mem_as_source)
            databus <= memory[mem_addr[2:0]];

        if (memory[6] == memory[7]) begin // 14��15�Ĵ������ʱ�����PWM�����ź�
            memory[5][0] <= 1'b1; // 13�Ĵ������λΪ1
        end
        memory[7] <= cnt;
    end
    assign mem_addr = inbus;

endmodule

module IR (
    // ϵͳ�ź�
    input         clk,
    
    // ˫���������߽ӿ�
    output reg  [7:0]  databus,
    input  [7:0]  inbus,        // �����������ߣ�����ALU������ģ�飩
    
    // �����ź�
    input         ir_in,        // �����߼���ָ���������Ч��
    
    // �������
    output [3:0]  opcode,       // ��4λ������ -> CU������3λ�����룩
    output [3:0]  operand       // ��4λ������ -> CU������5λ������Ϊ��������
);

    //----------------------------------
    // �ڲ��Ĵ�����״̬��
    //----------------------------------
    
    // ������������Ƶ�Ԫ
    assign opcode   = databus[7:4];
    assign operand  = databus[3:0];

    //----------------------------------
    // ͬ�������߼�
    //----------------------------------
    always @(posedge clk) begin
        // ָ����ؽ׶�
        if (ir_in) begin
            databus <= inbus;    // ������������
        end
    end

endmodule

module CU (
    // ʱ���븴λ
    input         clk,
    input         rst,          // ȫ�ָ�λ�ź�
    
    // ָ��ӿ�
    input  [3:0]  opcode,
    input  [3:0]  operand,

    // �����ַ���߿���
    output reg      pc_reset,       // PC����
    output reg        pc_out,         // PC��ַ���ʹ��
    output reg        pc_inc,         // PC����
    output reg        pc_jump,        // PC��ת
    
    // ָ��Ĵ������߿���
    output reg        iMDRout,        // iMDRд������

    // �Ĵ��������߿���
    // ALU���߿���
    output reg        ALUAin,         // ALU����A����
    output reg        ALUBin,         // ALU����B����
    output reg        ALUout,         // ALU�������
    output reg [2:0]  ALUop,          // ALU����ѡ��
    input             cf,             // ��λ��־��CF��
    input             zf,             // ���־��ZF��

    // ���ݴ洢�����߿���
    output reg        dmdr_iin,       // dMDR����������ʹ��
    output reg        dmdr_iout,      // dMDR���������ʹ�ܣ��������������
    output reg        mem_as_target,  // ���ݴ洢����ΪĿ���ַ����
    output reg        mem_as_source,

    output reg        ir_in,          // �����߼���ָ���������Ч��
    output reg        imm_out,        // ���������������λ��

    output reg [2:0]  s_addrbus,      // Դ��ַѡ��
    output reg [2:0]  t_addrbus,      // Ŀ���ַѡ��
    output reg [7:0]  cnt,
    output     [7:0]  next_cnt,
    input      rx, // UART�����ߣ���ѡ��
    output reg tx, // UART�����ߣ���ѡ��
    output reg reg_bus_in, // �Ĵ��������루R0-R3��
    output reg reg_en_in, // �Ĵ���������ʹ�ܣ�R0-R3��
    input  reg_bus_out, // �Ĵ����������R0-R3��
    output reg reg_en_out // �Ĵ��������ʹ�ܣ�R0-R3��
);
    reg imm_get_en; // ��������ȡʹ��
    reg [2:0] cur_sta;
    reg fetch1_wait; // �ȴ��ź�
    reg [1:0] imm_reg; // �������Ĵ���
    parameter 
        FETCH1   = 3'b000,
        FETCH2   = 3'b001,
        EXEC1    = 3'b010,
        EXEC2    = 3'b011,
        EXEC3    = 3'b100,
        EXEC4    = 3'b101;

	Incrementer cnt_inc(
		.op(cnt),
		.next_op(next_cnt)
	);
    always @(negedge clk) begin
        case (cur_sta)
            // ȡָ�׶�
            FETCH1: begin
                pc_out <= 1;            // PC�������������
                s_addrbus <= 3'b110;    // ָ��洢����ȡָ��
                t_addrbus <= 3'b111;    // ��Ŀ���ַ
                if (fetch1_wait) begin
                    fetch1_wait <= 1'b0; // ȡָ�ȴ��źŸ�λ
                    cur_sta <= FETCH1; // ������FETCH1״̬
                end else begin
                    cur_sta <= FETCH2;    // ״̬ת�Ƶ�FETCH2
                end
            end
            FETCH2: begin
                pc_out <= 0;            // PCֹͣ�������
                s_addrbus <= 3'b111;    // ָֹͣ��洢����ȡָ��
                iMDRout <= 1;           // ָ�����ݴ洢���������������
                ir_in <= 1;             // ָ��Ĵ����������ж�ȡָ��
                pc_inc <= 1;            // PC����
                cur_sta <= EXEC1;    // ״̬ת�Ƶ�EXEC1
            end
            EXEC1: begin
                pc_inc <= 0;            // PCֹͣ����
                iMDRout <= 0;           // ָ�����ݴ洢��ֹͣ�������������
                ir_in <= 0;             // ָ��Ĵ���ֹͣ��ȡָ��
                
                // Start
                case (opcode)
                    // move rx, ry step1(move)
                    4'b0000: begin
                        s_addrbus <= {1'b0, operand[3:2]};    // rx����������Ϣ����������
                        t_addrbus <= {1'b0, operand[1:0]};   // ry�������������ϵ�������Ϣ
                    end
                    // move rx, (ry) step1(���Ѱַ)
                    4'b0001: begin
                        s_addrbus <= {1'b0, operand[3:2]};    // rx����������Ϣ����������
                        dmdr_iin <= 1;                        // MDR������������
                    end
                    // move i ry
                    4'b0010: begin
                        imm_reg <= operand[1:0]; // ������������ȡ�ļĴ������
                        pc_out <= 1;                     // PC�������������
                        s_addrbus <= 3'b110;             // ָ��洢����ȡָ��
                        imm_get_en <= 1;              // ��������ȡʹ��
                    end
                    // jump
                    4'b0100: begin
                        case (operand[3:2])
                            // ry
                            2'b00: begin
                                pc_jump <= 1;               // PC��ת
                                s_addrbus <= {1'b0, operand[1:0]}; // ֱ����ת����ַ
                            end
                            // jc ry
                            2'b01: begin
                                if (cf) begin
                                    pc_jump <= 1;           // PC��ת
                                    s_addrbus <= {1'b0, operand[1:0]}; // ֱ����ת����ַ
                                end else begin
                                    pc_jump <= 0;           // PC����ת
                                end
                            end
                            // jz ry
                            2'b10: begin
                                if (zf) begin
                                    pc_jump <= 1;           // PC��ת
                                    s_addrbus <= {1'b0, operand[1:0]}; // ֱ����ת����ַ
                                end else begin
                                    pc_jump <= 0;           // PC����ת
                                end
                            end
                        endcase
                    end
                    4'b0101: begin
                        case(operand[3:2])
                        // in ry // ry <= uart receive
                        2'b00: begin
                            t_addrbus <= {1'b0, operand[1:0]};
                            reg_bus_in <= rx;               // �Ĵ���������
                            reg_en_in <= 1;                 // �Ĵ���������ʹ���ź�
                        end
                        // out ry // uart send
                        2'b01: begin
                            s_addrbus <= {1'b0, operand[1:0]};
                            reg_en_out <= 1;
                        end
                        endcase
                    end
                    // move (rx), ry step1(���Ѱַ)
                    4'b0110: begin
                        s_addrbus <= {1'b0, operand[3:2]};    // rx�����ַ��������������
                        mem_as_source <= 1;
                    end
                    // rtn step1(rtn)
                    4'b0111: begin
                        pc_reset <= 1;                       // PC����
                    end
                    // and/or/xor/add/sub/shl/shr rx ry step1(alua <= rx)
                    4'b1000, 4'b1001, 4'b1011, 4'b1101, 4'b1111, 4'b1100, 4'b1110: begin
                        s_addrbus <= {1'b0, operand[3:2]};
                        ALUAin <= 1;
                    end
                    default: cur_sta <= cur_sta;
                endcase
                cur_sta <= EXEC2;    // ״̬ת�Ƶ�EXEC2

            end
            EXEC2: begin
                if (imm_get_en) begin
                    pc_out <= 0;                       // PCֹͣ���
                    pc_inc <= 1;                       // PC����
                    s_addrbus <= 3'b111;                // ֹͣԴ��ַ��������
                    iMDRout <= 1;                  // ָ�����ݴ洢���������������
                    ir_in <= 1;                     // ָ��Ĵ����������ж�ȡָ��
                end else begin
                case (opcode)
                    // move rx, ry endstep
                    4'b0000: begin
                        s_addrbus <= 3'b111;
                        t_addrbus <= 3'b111;
                    end
                    // move rx, (ry) step2(move)
                    4'b0001: begin
                        dmdr_iin <= 0;                        // MDRֹͣ������������
                        s_addrbus <= {1'b0, operand[1:0]};    // ry���͵�ַ��Ϣ����������
                        // t_addrbus <= 3'b100;                 // ͨ���������ݵ�ַ��Ϣ����Ӧ�ڴ����ݴ���dMDR
                        mem_as_target <= 1;                // ���ݴ洢����ΪĿ���ַ����
                    end
                    // jump step2(jump)
                    4'b0100: begin
                        pc_jump <= 0;                       // PCֹͣ��ת
                        s_addrbus <= 3'b111;                // ֹͣԴ��ַ��������
                        t_addrbus <= 3'b111;                // ֹͣĿ���ַ��������
                    end
                    4'b0101: begin
                        case(operand[3:2])
                        // in ry step2(in)
                        2'b00: begin
                            t_addrbus <= 3'b111;                // ֹͣ�Ĵ���������
                            reg_en_in <= 0;                     // �Ĵ���������ʹ����
                        end
                        2'b01:begin
                            s_addrbus <= 3'b111;                // ֹͣ�Ĵ��������
                            reg_en_out <= 0;                    // �Ĵ��������ʹ����
                        end
                        endcase
                    end
                    // move (rx), ry step2(move)
                    4'b0110: begin
                        mem_as_source <= 0;
                        dmdr_iout <= 1;                         // dMDR�����������������
                        // s_addrbus <= 3'b100;                    // ������ݵ�ַ�������߷ô�
                        t_addrbus <= {1'b0, operand[1:0]}; // ry�������������ϵ�����
                    end
                    // rtn endstep
                    4'b0111: begin
                        pc_reset <= 0;                          // PCֹͣ����
                    end
                    // and rx ry step2(alub <= ry)
                    4'b1000: begin
                        s_addrbus <= {1'b0, operand[1:0]};
                        ALUAin <= 0;
                        ALUBin <= 1;
                        ALUop  <= 3'b100;
                    end
                    // or rx ry step2(alub <= ry)
                    4'b1001: begin
                        s_addrbus <= {1'b0, operand[1:0]};
                        ALUAin <= 0;
                        ALUBin <= 1;
                        ALUop  <= 3'b101;
                    end
                    // xor rx ry step2(alub <= ry)
                    4'b1011: begin
                        s_addrbus <= {1'b0, operand[1:0]};
                        ALUAin <= 0;
                        ALUBin <= 1;
                        ALUop  <= 3'b111;
                    end
                    // not ry step2(alub <= ry)
                    4'b1010:  begin
                        s_addrbus <= {1'b0, operand[1:0]};
                        ALUAin <= 0;
                        ALUBin <= 1;
                        ALUop  <= 3'b110;
                    end
                    // shl rx, ry (alub <= ry)
                    4'b1100: begin
                        s_addrbus <= {1'b0, operand[1:0]};
                        ALUAin <= 0;
                        ALUBin <= 1;
                        ALUop  <= 3'b010;
                    end
                    // and rx ry step2(alub <= ry)
                    4'b1101: begin
                        s_addrbus <= {1'b0, operand[1:0]};
                        ALUAin <= 0;
                        ALUBin <= 1;
                        ALUop  <= 3'b000;
                    end
                    // shr rx ry(alub <= ry)
                    4'b1110: begin
                        s_addrbus <= {1'b0, operand[1:0]};
                        ALUAin <= 0;
                        ALUBin <= 1;
                        ALUop  <= 3'b011;
                    end
                    // sub rx ry step2(alub <= ry)
                    4'b1111: begin
                        s_addrbus <= {1'b0, operand[1:0]};
                        ALUAin <= 0;
                        ALUBin <= 1;
                        ALUop  <= 3'b001;
                    end
                    default: cur_sta <= cur_sta;
                endcase
                end
                // ״̬ת�Ƶ�EXEC3
                cur_sta <= EXEC3;
            end
            EXEC3: begin
                if (imm_get_en) begin
                    pc_inc <= 0;                       // PCֹͣ����
                    iMDRout <= 0;                  // ָ�����ݴ洢��ֹͣ�������������
                    ir_in <= 0;                     // ָ��Ĵ���ֹͣ��ȡָ��
                    imm_out <= 1;                  // ���������ʹ��
                    t_addrbus <= {1'b0, imm_reg}; // ������д��Ĵ���
                end else begin
                case (opcode)
                    // move rx, (ry) endstep
                    4'b0001: begin
                        s_addrbus <= 3'b111;                      // ֹͣԴ��ַ��������  
                        t_addrbus <= 3'b111;                 // ֹͣĿ���ַ��������
                        mem_as_target <= 0;
                    end
                    4'b0101: begin
                        case(operand[3:2])
                        2'b01:begin
                            tx <= reg_bus_out;               // �Ĵ��������ʹ��
                        end
                        endcase
                    end
                    // move (rx), ry endstep
                    4'b0110: begin
                        dmdr_iout <= 0;                          // dMDRֹͣ�����������������
                        t_addrbus <= 3'b111;                     // ryֹͣ�������������ϵ�����
                        s_addrbus <= 3'b111;                     // ֹͣԴ��ַ��������
                    end
                    // and/or/xor/add/sub/shl/shr rx ry step3 (result <= databus)
                    4'b1000, 4'b1001, 4'b1010, 4'b1011, 4'b1101, 4'b1111, 4'b1100, 4'b1110: begin
                        //clear
                        s_addrbus <= 3'b111;
                        ALUBin <= 0;
                        //operate
                        ALUout <= 1;                         
                        t_addrbus <= {1'b0, operand[1:0]};        // ������д��ry
                    end
                    default: cur_sta <= cur_sta;
                    endcase
                // ״̬ת�Ƶ�EXEC4
                end
                cur_sta <= EXEC4;
            end
            EXEC4: begin
                if (imm_get_en) begin
                    imm_out <= 0; // ���������ֹͣ
                    imm_get_en <= 0; // ��������ȡʹ�ܸ�λ
                    t_addrbus <= 3'b111; // ֹͣĿ���ַ��������
                end else begin
                    case (opcode)
                    // (and/or/not/xor/add/sub/shl/shr rx ry) / add/sub i endstep
                    4'b1000, 4'b1001, 4'b1010, 4'b1011, 4'b1100, 4'b1101, 4'b1110, 4'b1111, 4'b1100, 4'b1110: begin
                        ALUout <= 0;                         
                        t_addrbus <= 3'b111;
                    end
                    default: cur_sta <= cur_sta;
                    endcase
                end
                // ״̬ת�Ƶ�FETCH1
                cur_sta <= FETCH1;
                if(cnt[3] & cnt[4] & cnt[6] & cnt[7]) cnt <= 0;
                // if(cnt == 19) cnt <= 0;
                // if (cnt[0]) cnt <= 0;
                else begin
                    if (~x) begin
                        if (~xx)
                            cnt <= next_cnt;
                        xx = ~xx;
                    end
                    x = ~x;
                end
            end
            default: cur_sta <= FETCH1;
        endcase

        if (rst) begin
            cur_sta <= FETCH1; // ��λʱ״̬���ص�FETCH1
            pc_reset <= 1'b0;
            pc_out <= 1'b0; // PC���ֹͣ
            pc_inc <= 1'b0; // PC����ֹͣ
            iMDRout <= 1'b0; // ָ��洢�����ֹͣ
            ALUAin <= 1'b0; // ALU A����ֹͣ
            ALUBin <= 1'b0; // ALU B����ֹͣ
            ALUout <= 1'b0; // ALU���ֹͣ
            ALUop <= 3'b000; // ALU����������
            dmdr_iin <= 1'b0; // ���ݴ洢������ֹͣ
            dmdr_iout <= 1'b0; // ���ݴ洢�����ֹͣ
            ir_in <= 1'b0; // ָ��Ĵ�������ֹͣ
            imm_out <= 1'b0; // ���������ֹͣ
            s_addrbus <= 3'b111; // Դ��ַ�����ÿ�
            t_addrbus <= 3'b111; // Ŀ���ַ�����ÿ�
            fetch1_wait <= 1'b1; // ����ȴ�״̬
            cnt <= 0;
            tx <= 1;
        end
    end

    reg x = 0;
    reg xx = 0;
    
endmodule

module Incrementer(
    input [7:0]  op,
    output [7:0] next_op
);
    // ���ɽ�λ��
    wire [7:0] carry;
    
    // ���λ�Ľ�λ��1����Ϊ����Ҫ��1��
    assign carry[0] = 1'b1;
    
    // �������λλ
    assign carry[1] = op[0] & carry[0];
    assign carry[2] = op[1] & carry[1];
    assign carry[3] = op[2] & carry[2];
    assign carry[4] = op[3] & carry[3];
    assign carry[5] = op[4] & carry[4];
    assign carry[6] = op[5] & carry[5];
    assign carry[7] = op[6] & carry[6];
    
    // �����������
    // next_op = op + 1 = op ^ carry ^ {carry[6:0], 1'b0}
    assign next_op = op ^ carry;
endmodule

