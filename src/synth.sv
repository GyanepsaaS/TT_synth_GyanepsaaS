`default_nettype none

//Overall structure:
//synth calls soundproc and dac
//  soundproc calls signal_gen, envelope_gen, and amplitude_modulator
//      signal_gen generates signal
//      envelope_gen modulates signal with ADSR envelope
//      amplitude_modulator modulates signal with envelope
//  dac converts to analogue

module tt_um_synth_GyanepsaaS
    (input  wire [7:0] ui_in,    // Dedicated inputs - connected to the input switches
    output wire [7:0] uo_out,   // Dedicated outputs - connected to the 7 segment display
    input  wire [7:0] uio_in,   // IOs: Bidirectional Input path
    output wire [7:0] uio_out,  // IOs: Bidirectional Output path
    output wire [7:0] uio_oe,
    input  wire       ena,      // will go high when the design is enabled
    input  wire       clk,      // clock
    input  wire       rst_n     // reset_n - low to reset
    );

    logic not_rst;
    logic signed  [11:0] dout;

    assign uio_oe = 8'b0;

    //zeroing unused
    assign uio_out = 8'b0;
    assign uo_out[7:1] = 7'b0;

    assign not_rst = ~rst_n;
    soundproc sound1 (.waveform_enable(ui_in[0]), .main_clk(clk), .sample_clk(clk),
                        .rst(not_rst), .hold(ui_in[3]), .tone_freq_bin(ui_in[7:4]),
                        .attack(uio_in[7:6]), .decay(uio_in[5:4]),
                      .sustain(uio_in[3:2]), .rel(uio_in[1:0]), .dout(dout));

    dac dac1 (.din(dout), .clk(clk), .dout(uo_out[0]));


endmodule: tt_um_synth_GyanepsaaS

//Overall sound processing module
//Combines signal generation and amplitude modulating it with envelope
module soundproc
  #(parameter OUTPUT_BITS = 12,
  parameter FREQ_BITS = 4,
  parameter ACCUMULATOR_BITS = 24,
  parameter SAMPLE_CLK_FREQ = 10000000)

  (input [FREQ_BITS-1:0] tone_freq_bin,
  input waveform_enable,
  input main_clk,
  input sample_clk,
  input rst,
  output logic signed [OUTPUT_BITS-1:0] dout,

  // envelope generator params
  input logic hold,
  input [1:0] attack,
  input [1:0] decay,
  input [1:0] sustain,
  input [1:0] rel
);
  logic signed [OUTPUT_BITS-1:0] tone_generator_data;
  logic[7:0] envelope_amplitude;
  logic accumulator_msb;

  signal_gen #(
    .FREQ_BITS(FREQ_BITS),
    .OUTPUT_BITS(OUTPUT_BITS),
    .ACCUMULATOR_BITS(ACCUMULATOR_BITS)
  ) signal (
      .tone_freq_bin(tone_freq_bin),
      .en_sawtooth(waveform_enable),
      .en_triangle(~waveform_enable),
      .main_clk(main_clk),
      .sample_clk(sample_clk),
      .dout(tone_generator_data),
      .accumulator_msb(accumulator_msb)
    );

  envelope_gen #(
    .SAMPLE_CLK_FREQ(SAMPLE_CLK_FREQ)
  )
    envelope(
    .clk(sample_clk),
    .rst(rst),
    .hold(hold),
    .a(attack),
    .d(decay),
    .s(sustain),
    .r(rel),
    .amplitude(envelope_amplitude)
  );

  amplitude_modulator #(.DATA_BITS(OUTPUT_BITS)) modulator(
    .clk(sample_clk),
    .din(tone_generator_data),
    .amplitude(envelope_amplitude),
    .dout(dout)
  );

endmodule: soundproc


//Signal generation:
//Supports triangle and sawtooth wave
//Waves generated using phase accumulator
//reference: C64 SID - https://www.youtube.com/watch?v=CIo93AE8Fsw

module signal_gen
    #(parameter FREQ_BITS = 4,
    parameter OUTPUT_BITS = 12,
    parameter ACCUMULATOR_BITS = 24)
    (input [FREQ_BITS-1:0] tone_freq_bin,
    input main_clk,
    input sample_clk,
    output logic signed [OUTPUT_BITS-1:0] dout,
    output logic accumulator_msb,

    input en_triangle,
    input en_sawtooth);

    logic [ACCUMULATOR_BITS-1:0] accumulator;
    logic [ACCUMULATOR_BITS-1:0] prev_accumulator;
    logic tone_freq;
    logic dout_tmp;
    
    assign tone_freq = 1<<tone_freq_bin;
    assign accumulator_msb = accumulator[ACCUMULATOR_BITS-1];

    logic [OUTPUT_BITS-1:0] triangle_dout;
    triangle
        #(.ACCUMULATOR_BITS(ACCUMULATOR_BITS),
        .OUTPUT_BITS(OUTPUT_BITS))
        t1 (.accumulator(accumulator),
            .dout(triangle_dout));

    logic [OUTPUT_BITS-1:0] sawtooth_dout;
    sawtooth
        #(.ACCUMULATOR_BITS(ACCUMULATOR_BITS),
        .OUTPUT_BITS(OUTPUT_BITS))
        s1 (.accumulator(accumulator),
            .dout(sawtooth_dout));

    always @(posedge main_clk) begin
        prev_accumulator <= accumulator;
        accumulator <= accumulator + tone_freq;
    end

    always @(posedge sample_clk) begin
        dout_tmp = (2**OUTPUT_BITS)-1;
        if (en_sawtooth)
            dout_tmp = dout_tmp & sawtooth_dout;
        if (en_triangle)
            dout_tmp = dout_tmp & triangle_dout;
    end

    assign dout = dout_tmp ^ (2**(OUTPUT_BITS-1)); //signed

endmodule: signal_gen

//sawtooth wave
module sawtooth
    #(parameter ACCUMULATOR_BITS = 24,
    parameter OUTPUT_BITS = 12)
    (input [ACCUMULATOR_BITS-1:0] accumulator,
    output logic [OUTPUT_BITS-1:0] dout);

    assign dout = accumulator[ACCUMULATOR_BITS-1 -: OUTPUT_BITS];

endmodule: sawtooth

//triangle wave
module triangle
    #(parameter ACCUMULATOR_BITS = 24,
    parameter OUTPUT_BITS = 12)
    (input [ACCUMULATOR_BITS-1:0] accumulator,
    output logic [OUTPUT_BITS-1:0] dout);

    logic peak_reached; //reached max value, ramp down

    assign peak_reached = accumulator[ACCUMULATOR_BITS-1];
    assign dout = peak_reached ? ~accumulator[ACCUMULATOR_BITS-2 -: OUTPUT_BITS]
                : accumulator[ACCUMULATOR_BITS-2 -: OUTPUT_BITS];

endmodule: triangle



// envelope generation
// uses FSMD to find states
module envelope_gen #(
  parameter SAMPLE_CLK_FREQ = 10000000,
  parameter ACCUMULATOR_BITS = 26
)
(
  input clk,
  input hold,
  input [1:0] a,
  input [1:0] d,
  input [1:0] s,
  input [1:0] r,
  output logic [7:0] amplitude,
  input rst);

  localparam  ACCUMULATOR_SIZE = 2**ACCUMULATOR_BITS;
  localparam  ACCUMULATOR_MAX  = ACCUMULATOR_SIZE-1;

  logic [ACCUMULATOR_BITS:0] accumulator;
  logic [16:0] accumulator_inc;  /* value to add to accumulator */


  // calculate the amount to add to the accumulator each clock cycle to
  // achieve a full-scale value in n number of seconds. (n can be fractional seconds)
  `define CALCULATE_PHASE_INCREMENT(n) $rtoi(ACCUMULATOR_SIZE / (n * SAMPLE_CLK_FREQ))

  function [16:0] attack_table;
    input [1:0] param;
    begin
      case(param)
        2'b00: attack_table = `CALCULATE_PHASE_INCREMENT(0.005);
        2'b01: attack_table = `CALCULATE_PHASE_INCREMENT(0.050);
        2'b10: attack_table = `CALCULATE_PHASE_INCREMENT(0.500);
        2'b11: attack_table = `CALCULATE_PHASE_INCREMENT(5.000);
        default: attack_table = 65535;
      endcase
    end
  endfunction

  function [16:0] decay_release_table;
    input [1:0] param;
    begin
      case(param)
        2'b00: decay_release_table = `CALCULATE_PHASE_INCREMENT(0.005);
        2'b01: decay_release_table = `CALCULATE_PHASE_INCREMENT(0.050);
        2'b10: decay_release_table = `CALCULATE_PHASE_INCREMENT(0.500);
        2'b11: decay_release_table = `CALCULATE_PHASE_INCREMENT(10.000);
        default: decay_release_table = 65535;
      endcase
    end
  endfunction

  localparam OFF     = 3'd0;
  localparam ATTACK  = 3'd1;
  localparam DECAY   = 3'd2;
  localparam SUSTAIN = 3'd3;
  localparam RELEASE = 3'd4;

  logic[2:0] state;

  initial begin
    state = OFF;
    amplitude = 0;
    accumulator = 0;
  end


  // value adding to accumulator for attack
  logic [16:0] attack_inc;
  always @(a) begin
    attack_inc <= attack_table(a); // convert 4-bit value into phase increment amount
  end

 //value adding to accumulator for decay
  logic [16:0] decay_inc;
  always @(d) begin
      decay_inc <= decay_release_table(d); // convert 4-bit value into phase increment amount
  end

  logic [7:0] sustain_volume;  // 2-bit volume expanded into an 8-bit value
  logic [7:0] sustain_gap;     // gap between sustain-volume and full-scale (255)
                             // used to calculate decay phase scale factor

  assign sustain_volume = { s, 6'b0000 };
  assign sustain_gap = 255 - sustain_volume;

  // value to add to accumulator during release (reduce)
  logic [16:0] release_inc;
  always @(r) begin
      release_inc <= decay_release_table(r); // convert 2-bit value into phase increment amount
  end

  logic [16:0] dectmp;
  logic [16:0] reltmp;


  logic [7:0] exp_out;  // exponential decay mapping of accumulator output; used for decay and release cycles
  eight_bit_exponential_decay_lookup exp_lookup(.din(accumulator[ACCUMULATOR_BITS-1 -: 8]), .dout(exp_out));

//envelope generator next state
  function [2:0] next_state;
    input [2:0] s;
    input g;
    begin
      case ({ s, g })
        { ATTACK,  1'b0 }: next_state = RELEASE;
        { ATTACK,  1'b1 }: next_state = DECAY;
        { DECAY,   1'b0 }: next_state = RELEASE;
        { DECAY,   1'b1 }: next_state = SUSTAIN;
        { SUSTAIN, 1'b0 }: next_state = RELEASE;
        { SUSTAIN, 1'b1 }: next_state = SUSTAIN;
        { RELEASE, 1'b0 }: next_state = OFF;
        { RELEASE, 1'b1 }: next_state = ATTACK;
        { OFF,     1'b0 }: next_state = OFF;
        { OFF,     1'b1 }: next_state = ATTACK;
        default: next_state = OFF;  /* default is end (off) state */
      endcase
    end
  endfunction

  logic overflow;
  assign overflow = accumulator[ACCUMULATOR_BITS];

  logic prev_hold;

  always @(posedge clk)
    begin

      /* check for hold low->high transitions (straight to attack phase)*/
      prev_hold <= hold;
      if (hold && !prev_hold)
        begin
          accumulator <= 0;
          state <= ATTACK;
        end

      /* otherwise, flow through ADSR state machine */
      if (overflow)
        begin
          accumulator <= 0;
          dectmp <= 8'd255;
          state <= next_state(state, hold);
        end
      else begin
        case (state)
          ATTACK:
            begin
              accumulator <= accumulator + attack_inc;
              amplitude <= accumulator[ACCUMULATOR_BITS-1 -: 8];
            end
          DECAY:
            begin
              accumulator <= accumulator + decay_inc;
              dectmp <= ((exp_out * sustain_gap) >> 8) + sustain_volume;
              amplitude <= dectmp;
            end
          SUSTAIN:
          begin
            amplitude <= sustain_volume;
            state <= next_state(state, hold);
          end
          RELEASE:
            begin
              accumulator <= accumulator + release_inc;
              reltmp <= ((exp_out * sustain_volume) >> 8);
              amplitude <= reltmp;
              if (hold) begin
                amplitude <= 0;
                accumulator <= 0;
                state <= next_state(state, hold);
              end
            end
          default:
            begin
              amplitude <= 0;
              accumulator <= 0;
              state <= next_state(state, hold);
            end
        endcase
    end
  end
endmodule: envelope_gen

module amplitude_modulator
    #(parameter DATA_BITS = 12,
    parameter AMPLITUDE_BITS = 8)
    (input signed [DATA_BITS-1:0] din,
    input [AMPLITUDE_BITS-1:0] amplitude,
    input clk,
    output logic signed [DATA_BITS-1:0] dout);

  logic signed [AMPLITUDE_BITS:0] amp_signed;
  assign amp_signed = { 1'b0, amplitude[AMPLITUDE_BITS-1:0] }; // amplitude with extra MSB (0)

  logic signed [DATA_BITS+AMPLITUDE_BITS-1:0] scaled_din;  // intermediate value with extended precision

  always @(posedge clk) begin
    scaled_din <= (din * amp_signed);
  end

  assign dout = scaled_din[DATA_BITS+AMPLITUDE_BITS-1 -: DATA_BITS];

endmodule: amplitude_modulator


//DAC - pdm
module dac
    #(parameter DATA = 12)
    (input signed [DATA-1:0] din,
    input logic clk,
    output logic dout);

logic [DATA:0] accumulator;
logic [DATA-1:0] u_in;

assign u_in = din ^ (2**(DATA-1));

always @(posedge clk) begin
  accumulator <= (accumulator[DATA-1 : 0] + u_in);
end

assign dout = accumulator[DATA];

endmodule: dac

module eight_bit_exponential_decay_lookup (
   input logic [7:0] din,
   output logic [7:0] dout);

    reg [0:7] exp_lookup [0:255];
    initial $readmemh("exp_lookup_table.rom", exp_lookup);

 assign dout = exp_lookup[din];

endmodule
