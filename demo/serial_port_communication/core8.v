module core8(
    input clk,
    input n_rst,
    // 保持信号，防止优化掉
    output r00,
    output r01,
    output r02,
    output r03,
    output r04,
    output r05,
    output r06,
    output r07,
    // 串口通信接口
    input rx, // 串口接收
    output tx // 串口发送
);
    wire rst = ~n_rst; // 低电平复位信号

    wire [7:0] databus;
    
    wire alua_in;
    wire alub_in;
    wire alu_out;
    wire[2:0] sel;
    wire cf;
    wire zf;
    wire pc_inc; // 程序计数器自增
    wire pc_jump; // 程序计数器跳转
    wire pc_out; // 程序计数器输出到总线
    wire pc_reset; // 程序计数器复位
    wire[2:0] s_addrbus; // 源寄存器地址总线
    wire[2:0] t_addrbus; // 目标寄存器地址总线 
    wire iMDRout;
    wire dmdr_iin; // 数据存储器输入
    wire dmdr_iout; // 数据存储器输出、
    wire ir_in; // 指令寄存器输入
    wire imm_out; // 立即数输出
    wire[3:0] opcode; // 操作码
    wire[3:0] operand; // 操作数
    wire[7:0] cnt;
    wire[7:0] r3dbg;
    
    
    // 各部件数据总线
    wire [7:0] alu_bus;
    wire [7:0] pc_bus;
    wire [7:0] ir_bus;
    wire [7:0] reg_bus[0:3];
    wire cur_reg_bus_in; // 控制单元寄存器组输出
    wire cur_reg_en_in; // 控制单元寄存器组使能信号
    wire cur_reg_bus_out; // 当前寄存器组输出
    wire cur_reg_en_out; // 当前寄存器组使能信号
    wire [7:0] imem_bus;
    wire [7:0] dmem_bus;
    reg [3:0] cur_send_sta = 4'b0000; // 当前发送状态寄存器
    wire mem_as_target; // 数据存储器作为目标地址总线
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

    // 对databus多路复用
    assign databus = 
        (pc_out) ? pc_bus :          // 程序计数器输出
        (iMDRout) ? imem_bus :      // 指令存储器输出
        (dmdr_iout) ? dmem_bus :    // 数据存储器输出
        (imm_out) ? ir_bus :          // 指令寄存器输入
        (alu_out) ? alu_bus :       // ALU输出
        s_addrbus[2] == 1'b0 ? reg_bus[{s_addrbus[1:0]}] : // 源寄存器组输出
        8'bzzzz_zzzz;                    // 寄存器组输出

    // Instantiate ALU
    ALU alu_inst(
        .databus(alu_bus),
        .inbus(databus),
        .clk(clk),
        .alua_in(alua_in),
        .alub_in(alub_in),
        .sel(sel),
        .cf(cf),
        .zf(zf) // 零标志（ZF）
    );


    //----------------------------------
    // 子模块实例化
    //----------------------------------
    // 程序计数器
    PC u_PC (
        .clk(clk),
        .pc_inc(pc_inc),
        .pc_jump(pc_jump),
        .databus(pc_bus),
        .inbus(databus), // 输入数据总线（来自ALU或其他模块）
        .reset(pc_reset | rst)
    );
    // 通用寄存器
    reg_file u_reg_file (
        .clk(clk),
        .rst(rst | pc_reset), // 全局复位信号
        .t_addrbus(t_addrbus),
        .s_addrbus(s_addrbus),
        .databus0(reg_bus[0]),
        .databus1(reg_bus[1]),
        .databus2(reg_bus[2]),
        .databus3(reg_bus[3]),
        .inbus(databus),
        .cur_reg_bus_in(cur_reg_bus_in), // 控制单元寄存器组输出
        .cur_reg_en_in(cur_reg_en_in), // 控制单元寄存器组使能信号 
        .cur_reg_bus_out(cur_reg_bus_out), // 当前寄存器组输出
        .cur_reg_en_out(cur_reg_en_out), // 当前寄存器组使能信号
        .r3dbg(r3dbg)
    );

    // 指令存储器
    inst_mem u_inst_mem (
        .clk(clk),
        .pc_addr(databus),      // 从总线获取地址
        .databus(imem_bus)      // 输出到总线
    );

    // 数据存储器
    DataMemory u_data_memory (
        .clk(clk),
        // .rst(rst),
        .mem_as_target(mem_as_target), // 数据存储器作为目标地址总线
        .mem_as_source(mem_as_source),
        .databus(dmem_bus),      // 数据总线
        .inbus(databus), // 输入数据总线（来自ALU或其他模块）
        .dmdr_iin(dmdr_iin),    // 输入使能
        .cnt(cnt) // 计数器输出（用于UART发送）
    );

    // 指令寄存器
    IR u_IR (
        .clk(clk),
        .databus(ir_bus),
        .inbus(databus), // 输入数据总线（来自ALU或其他模块）
        .ir_in(ir_in),
        .opcode(opcode),              // 内部连接至CU
        .operand(operand)              // 内部连接至CU
    );

        // 控制单元
    CU u_CU (
        .clk(clk),
        .rst(rst),         // 全局复位信号
        .opcode(opcode),   // 直接连接IR内部信号
        .operand(operand),
        .pc_reset(pc_reset),       // 共用复位
        .pc_out(pc_out),
        .pc_inc(pc_inc),
        .pc_jump(pc_jump), // PC跳转
        .iMDRout(iMDRout),
        .ALUAin(alua_in),
        .ALUBin(alub_in),
        .ALUout(alu_out),
        .ALUop(sel),
        .cf(cf),
        .zf(zf), // 零标志（ZF）
        .dmdr_iin(dmdr_iin),
        .dmdr_iout(dmdr_iout),
        .mem_as_target(mem_as_target), // 数据存储器作为目标地址总线
        .mem_as_source(mem_as_source),
        .ir_in(ir_in),
        .imm_out(imm_out),
        .s_addrbus(s_addrbus),
        .t_addrbus(t_addrbus),
        .cnt(cnt),
        .rx(rx), // 串口接收
        .tx(tx), // 串口发送
        .reg_bus_in(cur_reg_bus_in),  // 寄存器组输出
        .reg_en_in(cur_reg_en_in),    // 寄存器组使能信号
        .reg_bus_out(cur_reg_bus_out), // 当前寄存器组输出
        .reg_en_out(cur_reg_en_out)  // 当前寄存器组使能信号
    );

endmodule

//PC
module PC(
    input         clk,
    input         pc_inc,       // PC是否自增
    input         pc_jump,      // PC是否跳转
    input         reset,
    input      [7:0] inbus,        // 输入数据总线
    output reg [7:0] databus       // 统一数据总线
);
    // reg [7:0] cur_pc;     // 当前程序计数器值

    //PC自增寄存器
    wire [7:0] next_pc;
    // 三态总线驱动（完全由外部控制）
    Incrementer PCInc(
        .op(databus),
        .next_op(next_pc)
    );
    // 输出到总线
    // assign databus = cur_pc; // 当pc_out为1时，输出当前PC值，否则为高阻态
    // PC核心逻辑
    always @(posedge clk or posedge reset) begin
        if (reset)              // PC置零
            databus <= 8'h00;
        else if (pc_inc)        // PC自增
            databus <= next_pc;
        else if (pc_jump) // PC跳转
            databus <= inbus;  // 跳转到指定地址
        else databus <= databus;  // PC不变（无信号）
    end
endmodule

//ALU
module ALU(databus,inbus,clk,alua_in,alub_in,sel,cf,zf);
	output reg [7:0] databus;       // 统一数据总线
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
    input [2:0]  sel,    // 扩展为3位操作码
    output reg [7:0] result,
    output reg       cf,
    output reg       zf // 零标志（ZF）
);
    
    // 加法器接口
    reg [7:0] adder_op1;
    reg [7:0] adder_op2;
    reg suben;
    wire [7:0] adder_sum;
    wire adder_cout;
    
    Adder u1(adder_op2, adder_op1, suben, adder_sum, adder_cout);
    
    always @(*) begin
        case(sel)
            // 算术运算
            3'b000, 3'b001: begin  // 加法/减法
                // 汇编代码其实是左源右目标，所以反过来放
                adder_op1 = op1;
                adder_op2 = op2;
                suben = sel[0];
                result = adder_sum;
                cf = adder_cout;
            end
            3'b010: begin  // 左移
                adder_op1 = 8'b0;
                adder_op2 = 8'b0;
                suben = 1'b0; // 左移不需要减法
                cf = 1'b0; // 左移不产生进位
                // if(adder_op1[7] | adder_op1[6] | adder_op1[5] | adder_op1[4] | adder_op1[3]) begin
                //     result = 8'b0; // 左移后全零
                // end else begin
                result = op2 << op1;
                // end
            end
            3'b011: begin  // 右移
                adder_op1 = 8'b0;
                adder_op2 = 8'b0;
                suben = 1'b0; // 左移不需要减法
                cf = 1'b0; // 左移不产生进位
                // if(adder_op1[7] | adder_op1[6] | adder_op1[5] | adder_op1[4] | adder_op1[3]) begin
                //     result = 8'b0; // 右移后全零
                // end else begin
                result = op2 >> op1;
                // end
            end
            // 逻辑运算
            3'b100: begin  // 与（AND）
				adder_op1 = 8'b0;
                adder_op2 = 8'b0;
                suben = 1'b0;
                result = op1 & op2;
                cf = 1'b0;  // 逻辑运算清零CF
            end
            3'b101: begin  // 或（OR）
				adder_op1 = 8'b0;
                adder_op2 = 8'b0;
                suben = 1'b0;
                result = op1 | op2;
                cf = 1'b0;
            end
            3'b110: begin  // 非（NOT）
				adder_op1 = 8'b0;
                adder_op2 = 8'b0;
                suben = 1'b0;
                result = op2 ^ 8'hFF; // 取反
                cf = 1'b0;
            end
            3'b111: begin  // 异或（XOR）
				adder_op1 = 8'b0;
                adder_op2 = 8'b0;
                suben = 1'b0;
                result = op1 ^ op2;
                cf = 1'b0;
            end
        endcase
        zf = (result == 8'b0); // 零标志（ZF）为1表示结果为0
    end
endmodule


module Adder(
    input [7:0]  op1,     // 操作数1
    input [7:0]  op2,     // 操作数2
    input        suben,   // 减法使能（1=减法）
    output [7:0] sum,     // 和/差结果
    output       co       // 进位/借位
);
    // 减法转换为补码加法
    wire [7:0] adj_op2 = op2 ^ {8{suben}};
    wire cin = suben;     // 减法时cin=1（补码+1）

    // 生成位（Generate）和传播位（Propagate）
    wire [7:0] G = op1 & adj_op2;
    wire [7:0] P = op1 ^ adj_op2;

    // 超前进位计算（分组优化）
    wire [7:0] C;
    assign C[0] = cin;
    assign C[1] = G[0] | (P[0] & C[0]);
    assign C[2] = G[1] | (P[1] & G[0]) | (P[1] & P[0] & C[0]);
    assign C[3] = G[2] | (P[2] & G[1]) | (P[2] & P[1] & G[0]) | (P[2] & P[1] & P[0] & C[0]);
    
    // 4位组间进位（关键优化点）
    wire C4 = G[3] | (P[3] & G[2]) | (P[3] & P[2] & G[1]) | (P[3] & P[2] & P[1] & G[0]) | 
              (P[3] & P[2] & P[1] & P[0] & C[0]);
    
    assign C[4] = C4;
    assign C[5] = G[4] | (P[4] & C4);
    assign C[6] = G[5] | (P[5] & G[4]) | (P[5] & P[4] & C4);
    assign C[7] = G[6] | (P[6] & G[5]) | (P[6] & P[5] & G[4]) | (P[6] & P[5] & P[4] & C4);

    // 最终进位和结果
    assign co = suben ^ (G[7] | (P[7] & G[6]) | (P[7] & P[6] & G[5]) | 
                (P[7] & P[6] & P[5] & G[4]) | (P[7] & P[6] & P[5] & P[4] & C4));
    assign sum = P ^ C;
endmodule

//Register File
module reg_file(
    input         clk,
    input       rst,
    // 双地址总线接口
    input  [2:0]  t_addrbus,    // 目标地址总线
	input  [2:0]  s_addrbus,
    
    output reg [7:0]  databus0, // 双向数据总线（dMDR与寄存器组之间交互线）
    output reg [7:0]  databus1, // 双向数据总线（dMDR与寄存器组之间交互线）
    output reg [7:0]  databus2, // 双向数据总线（dMDR与寄存器组之间交互线）
    output reg [7:0]  databus3, // 双向数据总线（dMDR与寄存器组之间交互线）
    input [7:0]  inbus,  // 输入数据总线（来自ALU或其他模块）
    // 控制信号
    input cur_reg_bus_in, // 控制单元寄存器组输出
    input cur_reg_en_in, // 控制单元寄存器组使能信号
    output reg cur_reg_bus_out, // 当前寄存器组输出
    input cur_reg_en_out, // 当前寄存器组使能信号
    output [7:0] r3dbg
);
    assign r3dbg = databus3;
    
    //------------------------------------------
    // 同步写逻辑
    //------------------------------------------
    always @(negedge clk) begin
        // 目标寄存器写入（地址有效时）
        if (cur_reg_en_in) begin
            case (t_addrbus[1:0]) // 低2位选择寄存器
                2'd0: databus0[7] <= cur_reg_bus_in; // R0
                2'd1: databus1[7] <= cur_reg_bus_in; // R1
                2'd2: databus2[7] <= cur_reg_bus_in; // R2
                2'd3: databus3[7] <= cur_reg_bus_in; // R3
            endcase
        end
        else if (cur_reg_en_out) begin
            case(s_addrbus[1:0]) // 低2位选择寄存器
                2'd0: cur_reg_bus_out <= databus0[0]; // R0
                2'd1: cur_reg_bus_out <= databus1[0]; // R1
                2'd2: cur_reg_bus_out <= databus2[0]; // R2
                2'd3: cur_reg_bus_out <= databus3[0]; // R3
            endcase
        end
        else if (t_addrbus[2] == 1'b0) begin  // 最高位=0时有效
            case (t_addrbus[1:0]) // 低2位选择寄存器
                2'd0: databus0 <= inbus; // R0
                2'd1: databus1 <= inbus; // R1
                2'd2: databus2 <= inbus; // R2
                2'd3: databus3 <= inbus; // R3
            endcase
        end

        if (rst) begin
            // 全部寄存器复位为0
            databus0 <= 8'b0;
            databus1 <= 8'b0;
            databus2 <= 8'b0;
            databus3 <= 8'b0;
        end
    end
endmodule

//指令存储器
module inst_mem(
    input         clk,
    input  [7:0]  pc_addr,       // 程序计数器地址输入
    output  reg [7:0]  databus       // 数据总线输出
);
    // 存储器配置 - 明确为ROM
    (* romstyle = "M9K" *) reg [7:0] memory [0:255];

	initial begin
		$readmemb("uart_ic.hex", memory); // 覆盖初始化
	end
    
    // 同步读取
    always @(posedge clk) begin
        databus <= memory[pc_addr]; // 输出到总线
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
    // 双地址总线接口
    input  mem_as_source,
    // input  [2:0]  t_addrbus,    // 目标地址总线
    input  mem_as_target,
    output reg  [7:0]  databus,      // 双向数据总线（dMDR与数据总线之间交互寄存器）
    input [7:0] inbus, // 输入到dMDR的数据总线（来自ALU或其他模块）
    // 控制信号
    input       dmdr_iin,     // dMDR和总线输入使能
    input [7:0] cnt // 计数器输出（用于UART发送） - 直接连接到DataMemory
);

    (* ramstyle = "no_rw_check, logic" *)  // 禁用读写冲突检查
    reg [7:0] memory [0:7];


    // DataMemory实例
    wire [7:0] mem_dout;
    wire        mem_we;
    wire [7:0] mem_addr; // 内存地址寄存器

    // 写使能逻辑：t_addrbus==100时写入
    // wire mem_as_target = (t_addrbus == 3'b100);
    // wire mem_as_source = (s_addrbus == 3'b100);

    // assign mem_addr = inbus; // 地址寄存器更新
    // assign mem_din = mem_as_target ? inbus : 8'bzzzz_zzzz; // 写入数据总线（高阻态）

    // MDR与内存、总线的数据流控制
    always @(negedge clk) begin
        // 总线输入到MDR
        if (dmdr_iin)
            databus <= inbus;
        // MDR写到内存
        else if (mem_as_target)
            memory[mem_addr[2:0]] <= databus;
        // 内存读到MDR
        else if (mem_as_source)
            databus <= memory[mem_addr[2:0]];

        if (memory[6] == memory[7]) begin // 14和15寄存器相等时，输出PWM调光信号
            memory[5][0] <= 1'b1; // 13寄存器最低位为1
        end
        memory[7] <= cnt;
    end
    assign mem_addr = inbus;

endmodule

module IR (
    // 系统信号
    input         clk,
    
    // 双向数据总线接口
    output reg  [7:0]  databus,
    input  [7:0]  inbus,        // 输入数据总线（来自ALU或其他模块）
    
    // 控制信号
    input         ir_in,        // 从总线加载指令（上升沿有效）
    
    // 译码输出
    output [3:0]  opcode,       // 高4位操作码 -> CU（存在3位操作码）
    output [3:0]  operand       // 低4位操作数 -> CU（存在5位操作数为立即数）
);

    //----------------------------------
    // 内部寄存器与状态机
    //----------------------------------
    
    // 持续输出到控制单元
    assign opcode   = databus[7:4];
    assign operand  = databus[3:0];

    //----------------------------------
    // 同步控制逻辑
    //----------------------------------
    always @(posedge clk) begin
        // 指令加载阶段
        if (ir_in) begin
            databus <= inbus;    // 锁存总线数据
        end
    end

endmodule

module CU (
    // 时钟与复位
    input         clk,
    input         rst,          // 全局复位信号
    
    // 指令接口
    input  [3:0]  opcode,
    input  [3:0]  operand,

    // 程序地址总线控制
    output reg      pc_reset,       // PC清零
    output reg        pc_out,         // PC地址输出使能
    output reg        pc_inc,         // PC自增
    output reg        pc_jump,        // PC跳转
    
    // 指令寄存器总线控制
    output reg        iMDRout,        // iMDR写入总线

    // 寄存器组总线控制
    // ALU总线控制
    output reg        ALUAin,         // ALU数据A输入
    output reg        ALUBin,         // ALU数据B输入
    output reg        ALUout,         // ALU数据输出
    output reg [2:0]  ALUop,          // ALU操作选择
    input             cf,             // 进位标志（CF）
    input             zf,             // 零标志（ZF）

    // 数据存储器总线控制
    output reg        dmdr_iin,       // dMDR和总线输入使能
    output reg        dmdr_iout,      // dMDR和总线输出使能（数据总线输出）
    output reg        mem_as_target,  // 数据存储器作为目标地址总线
    output reg        mem_as_source,

    output reg        ir_in,          // 从总线加载指令（上升沿有效）
    output reg        imm_out,        // 立即数输出（低五位）

    output reg [2:0]  s_addrbus,      // 源地址选择
    output reg [2:0]  t_addrbus,      // 目标地址选择
    output reg [7:0]  cnt,
    output     [7:0]  next_cnt,
    input      rx, // UART接收线（可选）
    output reg tx, // UART发送线（可选）
    output reg reg_bus_in, // 寄存器组输入（R0-R3）
    output reg reg_en_in, // 寄存器组输入使能（R0-R3）
    input  reg_bus_out, // 寄存器组输出（R0-R3）
    output reg reg_en_out // 寄存器组输出使能（R0-R3）
);
    reg imm_get_en; // 立即数获取使能
    reg [2:0] cur_sta;
    reg fetch1_wait; // 等待信号
    reg [1:0] imm_reg; // 立即数寄存器
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
            // 取指阶段
            FETCH1: begin
                pc_out <= 1;            // PC输出至数据总线
                s_addrbus <= 3'b110;    // 指令存储器读取指令
                t_addrbus <= 3'b111;    // 无目标地址
                if (fetch1_wait) begin
                    fetch1_wait <= 1'b0; // 取指等待信号复位
                    cur_sta <= FETCH1; // 保持在FETCH1状态
                end else begin
                    cur_sta <= FETCH2;    // 状态转移到FETCH2
                end
            end
            FETCH2: begin
                pc_out <= 0;            // PC停止总线输出
                s_addrbus <= 3'b111;    // 停止指令存储器读取指令
                iMDRout <= 1;           // 指令数据存储器输出数据至总线
                ir_in <= 1;             // 指令寄存器从总线中读取指令
                pc_inc <= 1;            // PC自增
                cur_sta <= EXEC1;    // 状态转移到EXEC1
            end
            EXEC1: begin
                pc_inc <= 0;            // PC停止自增
                iMDRout <= 0;           // 指令数据存储器停止输出数据至总线
                ir_in <= 0;             // 指令寄存器停止读取指令
                
                // Start
                case (opcode)
                    // move rx, ry step1(move)
                    4'b0000: begin
                        s_addrbus <= {1'b0, operand[3:2]};    // rx发送数据信息至数据总线
                        t_addrbus <= {1'b0, operand[1:0]};   // ry接受数据总线上的数据信息
                    end
                    // move rx, (ry) step1(间接寻址)
                    4'b0001: begin
                        s_addrbus <= {1'b0, operand[3:2]};    // rx发送数据信息至数据总线
                        dmdr_iin <= 1;                        // MDR接收总线数据
                    end
                    // move i ry
                    4'b0010: begin
                        imm_reg <= operand[1:0]; // 锁存立即数读取的寄存器编号
                        pc_out <= 1;                     // PC输出至数据总线
                        s_addrbus <= 3'b110;             // 指令存储器读取指令
                        imm_get_en <= 1;              // 立即数获取使能
                    end
                    // jump
                    4'b0100: begin
                        case (operand[3:2])
                            // ry
                            2'b00: begin
                                pc_jump <= 1;               // PC跳转
                                s_addrbus <= {1'b0, operand[1:0]}; // 直接跳转到地址
                            end
                            // jc ry
                            2'b01: begin
                                if (cf) begin
                                    pc_jump <= 1;           // PC跳转
                                    s_addrbus <= {1'b0, operand[1:0]}; // 直接跳转到地址
                                end else begin
                                    pc_jump <= 0;           // PC不跳转
                                end
                            end
                            // jz ry
                            2'b10: begin
                                if (zf) begin
                                    pc_jump <= 1;           // PC跳转
                                    s_addrbus <= {1'b0, operand[1:0]}; // 直接跳转到地址
                                end else begin
                                    pc_jump <= 0;           // PC不跳转
                                end
                            end
                        endcase
                    end
                    4'b0101: begin
                        case(operand[3:2])
                        // in ry // ry <= uart receive
                        2'b00: begin
                            t_addrbus <= {1'b0, operand[1:0]};
                            reg_bus_in <= rx;               // 寄存器组输入
                            reg_en_in <= 1;                 // 寄存器组输入使能信号
                        end
                        // out ry // uart send
                        2'b01: begin
                            s_addrbus <= {1'b0, operand[1:0]};
                            reg_en_out <= 1;
                        end
                        endcase
                    end
                    // move (rx), ry step1(间接寻址)
                    4'b0110: begin
                        s_addrbus <= {1'b0, operand[3:2]};    // rx输出地址数据至数据总线
                        mem_as_source <= 1;
                    end
                    // rtn step1(rtn)
                    4'b0111: begin
                        pc_reset <= 1;                       // PC清零
                    end
                    // and/or/xor/add/sub/shl/shr rx ry step1(alua <= rx)
                    4'b1000, 4'b1001, 4'b1011, 4'b1101, 4'b1111, 4'b1100, 4'b1110: begin
                        s_addrbus <= {1'b0, operand[3:2]};
                        ALUAin <= 1;
                    end
                    default: cur_sta <= cur_sta;
                endcase
                cur_sta <= EXEC2;    // 状态转移到EXEC2

            end
            EXEC2: begin
                if (imm_get_en) begin
                    pc_out <= 0;                       // PC停止输出
                    pc_inc <= 1;                       // PC自增
                    s_addrbus <= 3'b111;                // 停止源地址发送数据
                    iMDRout <= 1;                  // 指令数据存储器输出数据至总线
                    ir_in <= 1;                     // 指令寄存器从总线中读取指令
                end else begin
                case (opcode)
                    // move rx, ry endstep
                    4'b0000: begin
                        s_addrbus <= 3'b111;
                        t_addrbus <= 3'b111;
                    end
                    // move rx, (ry) step2(move)
                    4'b0001: begin
                        dmdr_iin <= 0;                        // MDR停止接收总线数据
                        s_addrbus <= {1'b0, operand[1:0]};    // ry发送地址信息至数据总线
                        // t_addrbus <= 3'b100;                 // 通过总线数据地址信息将对应内存数据存至dMDR
                        mem_as_target <= 1;                // 数据存储器作为目标地址总线
                    end
                    // jump step2(jump)
                    4'b0100: begin
                        pc_jump <= 0;                       // PC停止跳转
                        s_addrbus <= 3'b111;                // 停止源地址发送数据
                        t_addrbus <= 3'b111;                // 停止目标地址接受数据
                    end
                    4'b0101: begin
                        case(operand[3:2])
                        // in ry step2(in)
                        2'b00: begin
                            t_addrbus <= 3'b111;                // 停止寄存器组输入
                            reg_en_in <= 0;                     // 寄存器组输入使能信
                        end
                        2'b01:begin
                            s_addrbus <= 3'b111;                // 停止寄存器组输出
                            reg_en_out <= 0;                    // 寄存器组输出使能信
                        end
                        endcase
                    end
                    // move (rx), ry step2(move)
                    4'b0110: begin
                        mem_as_source <= 0;
                        dmdr_iout <= 1;                         // dMDR输出数据至数据总线
                        // s_addrbus <= 3'b100;                    // 主存根据地址数据总线访存
                        t_addrbus <= {1'b0, operand[1:0]}; // ry接受数据总线上的数据
                    end
                    // rtn endstep
                    4'b0111: begin
                        pc_reset <= 0;                          // PC停止清零
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
                // 状态转移到EXEC3
                cur_sta <= EXEC3;
            end
            EXEC3: begin
                if (imm_get_en) begin
                    pc_inc <= 0;                       // PC停止自增
                    iMDRout <= 0;                  // 指令数据存储器停止输出数据至总线
                    ir_in <= 0;                     // 指令寄存器停止读取指令
                    imm_out <= 1;                  // 立即数输出使能
                    t_addrbus <= {1'b0, imm_reg}; // 立即数写入寄存器
                end else begin
                case (opcode)
                    // move rx, (ry) endstep
                    4'b0001: begin
                        s_addrbus <= 3'b111;                      // 停止源地址发送数据  
                        t_addrbus <= 3'b111;                 // 停止目标地址接受数据
                        mem_as_target <= 0;
                    end
                    4'b0101: begin
                        case(operand[3:2])
                        2'b01:begin
                            tx <= reg_bus_out;               // 寄存器组输出使能
                        end
                        endcase
                    end
                    // move (rx), ry endstep
                    4'b0110: begin
                        dmdr_iout <= 0;                          // dMDR停止输出数据至数据总线
                        t_addrbus <= 3'b111;                     // ry停止接受数据总线上的数据
                        s_addrbus <= 3'b111;                     // 停止源地址发送数据
                    end
                    // and/or/xor/add/sub/shl/shr rx ry step3 (result <= databus)
                    4'b1000, 4'b1001, 4'b1010, 4'b1011, 4'b1101, 4'b1111, 4'b1100, 4'b1110: begin
                        //clear
                        s_addrbus <= 3'b111;
                        ALUBin <= 0;
                        //operate
                        ALUout <= 1;                         
                        t_addrbus <= {1'b0, operand[1:0]};        // 计算结果写入ry
                    end
                    default: cur_sta <= cur_sta;
                    endcase
                // 状态转移到EXEC4
                end
                cur_sta <= EXEC4;
            end
            EXEC4: begin
                if (imm_get_en) begin
                    imm_out <= 0; // 立即数输出停止
                    imm_get_en <= 0; // 立即数获取使能复位
                    t_addrbus <= 3'b111; // 停止目标地址接受数据
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
                // 状态转移到FETCH1
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
            cur_sta <= FETCH1; // 复位时状态机回到FETCH1
            pc_reset <= 1'b0;
            pc_out <= 1'b0; // PC输出停止
            pc_inc <= 1'b0; // PC自增停止
            iMDRout <= 1'b0; // 指令存储器输出停止
            ALUAin <= 1'b0; // ALU A输入停止
            ALUBin <= 1'b0; // ALU B输入停止
            ALUout <= 1'b0; // ALU输出停止
            ALUop <= 3'b000; // ALU操作码清零
            dmdr_iin <= 1'b0; // 数据存储器输入停止
            dmdr_iout <= 1'b0; // 数据存储器输出停止
            ir_in <= 1'b0; // 指令寄存器输入停止
            imm_out <= 1'b0; // 立即数输出停止
            s_addrbus <= 3'b111; // 源地址总线置空
            t_addrbus <= 3'b111; // 目标地址总线置空
            fetch1_wait <= 1'b1; // 清除等待状态
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
    // 生成进位链
    wire [7:0] carry;
    
    // 最低位的进位是1（因为我们要加1）
    assign carry[0] = 1'b1;
    
    // 计算各进位位
    assign carry[1] = op[0] & carry[0];
    assign carry[2] = op[1] & carry[1];
    assign carry[3] = op[2] & carry[2];
    assign carry[4] = op[3] & carry[3];
    assign carry[5] = op[4] & carry[4];
    assign carry[6] = op[5] & carry[5];
    assign carry[7] = op[6] & carry[6];
    
    // 计算增量结果
    // next_op = op + 1 = op ^ carry ^ {carry[6:0], 1'b0}
    assign next_op = op ^ carry;
endmodule

