// testbench-only package: link fault-injection descriptor (spec 7.2)
package link_pkg;
  typedef enum logic [3:0] {
    FI_NONE = 0, FI_CORRUPT_BIT = 1, FI_DROP = 2, FI_DUPLICATE = 3, FI_SEQ_OFFSET = 4,
    FI_REPLACE_HDR = 5, FI_LINK_DOWN = 6, FI_EXTRA_DELAY = 7, FI_CORRUPT_LCRC = 8
  } fi_action_e;

  typedef struct packed {
    logic         enable;
    logic [3:0]   action;
    logic [3:0]   target;       // 0 any,1 TLP,2 DLLP,3 Ack,4 Nak,5 UpdateFC,6 InitFC,7 fmt/type,8 seq
    logic [1:0]   mode;         // 0 rate, 1 one-shot at index, 2 count
    logic [19:0]  rate_ppm;
    logic [31:0]  index;
    logic [31:0]  count;
    logic [31:0]  seed;
    logic [7:0]   match_fmt_type;
    logic [11:0]  match_seq;
    logic [11:0]  seq_offset;
    logic [127:0] hdr_xor;
    logic [15:0]  down_cycles;
    logic [15:0]  delay_cycles;
  } link_fi_t;
endpackage
