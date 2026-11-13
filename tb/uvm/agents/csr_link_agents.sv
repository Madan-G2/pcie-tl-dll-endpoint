// ============================================================================
// CSR agent, link monitor, link fault-injection agent, DL status monitor
// ============================================================================

// ---------------------------------------------------------------- CSR
typedef uvm_sequencer #(csr_item) csr_sequencer;

class csr_driver extends uvm_driver #(csr_item);
  `uvm_component_utils(csr_driver)
  virtual csr_if vif;
  function new(string n, uvm_component p); super.new(n, p); endfunction
  function void build_phase(uvm_phase phase);
    if (!uvm_config_db#(virtual csr_if)::get(this, "", "vif", vif)) `uvm_fatal("NOVIF", "csr_driver: vif not set")
  endfunction
  task run_phase(uvm_phase phase);
    realtime t_done = -1;
    vif.req <= 0; vif.we <= 0; vif.addr <= 0; vif.wdata <= 0;
    wait (vif.rst_n === 1'b1);
    forever begin
      seq_item_port.get_next_item(req);
      // CS-1 allows a request every cycle: an item that is ready in the same time step as the previous
      // acknowledge is driven on that edge (requests 2 cycles apart), otherwise wait for the next edge
      if ($realtime != t_done) @(posedge vif.clk);
      vif.req <= 1; vif.we <= req.we; vif.addr <= req.addr; vif.wdata <= req.wdata;
      @(posedge vif.clk); vif.req <= 0; vif.we <= 0;
      @(posedge vif.clk);
      if (!vif.ack) `uvm_error("CS-1", $sformatf("csr_ack not seen one cycle after the request (addr=%h)", req.addr))
      req.rdata = vif.rdata;
      seq_item_port.item_done();
      t_done = $realtime;
    end
  endtask
endclass

class csr_monitor extends uvm_monitor;
  `uvm_component_utils(csr_monitor)
  virtual csr_if vif;
  uvm_analysis_port #(csr_item) ap;
  function new(string n, uvm_component p); super.new(n, p); endfunction
  function void build_phase(uvm_phase phase);
    ap = new("ap", this);
    if (!uvm_config_db#(virtual csr_if)::get(this, "", "vif", vif)) `uvm_fatal("NOVIF", "csr_monitor: vif not set")
  endfunction
  task run_phase(uvm_phase phase);
    csr_item it;
    bit pend = 0;
    forever begin
      @(posedge vif.clk);
      if (pend) begin
        it.rdata = vif.rdata; it.t = cyc_now();
        if (!vif.ack) `uvm_error("CS-1", "csr_ack missing")
        ap.write(it); pend = 0;
      end
      if (vif.req) begin
        it = csr_item::type_id::create("csr_mon");
        it.we = vif.we; it.addr = vif.addr; it.wdata = vif.wdata; pend = 1;
      end
    end
  endtask
endclass

class csr_agent extends uvm_agent;
  `uvm_component_utils(csr_agent)
  csr_sequencer sqr; csr_driver drv; csr_monitor mon;
  function new(string n, uvm_component p); super.new(n, p); endfunction
  function void build_phase(uvm_phase phase);
    sqr = csr_sequencer::type_id::create("sqr", this);
    drv = csr_driver::type_id::create("drv", this);
    mon = csr_monitor::type_id::create("mon", this);
  endfunction
  function void connect_phase(uvm_phase phase);
    drv.seq_item_port.connect(sqr.seq_item_export);
  endfunction
endclass

// ---------------------------------------------------------------- link monitor (passive, one per tap)
class link_monitor extends uvm_monitor;
  `uvm_component_utils(link_monitor)
  virtual lnk_if vif;
  uvm_analysis_port #(link_frame) ap;
  int dir, tap_is_in;
  int unsigned n_tlp, n_dllp, n_bad_framing, n_aborted;
  virtual csr_if dl_vif;                          // status of the transmitting node (frames cut by a link-down)
  bit dl_prev = 1;
  function new(string n, uvm_component p); super.new(n, p); endfunction
  function void build_phase(uvm_phase phase);
    ap = new("ap", this);
    if (!uvm_config_db#(virtual lnk_if)::get(this, "", "vif", vif)) `uvm_fatal("NOVIF", "link_monitor: vif not set")
    if (!uvm_config_db#(int)::get(this, "", "dir", dir)) dir = 0;
    if (!uvm_config_db#(int)::get(this, "", "is_in", tap_is_in)) tap_is_in = 1;
    void'(uvm_config_db#(virtual csr_if)::get(this, "", "dl_vif", dl_vif));
  endfunction
  task run_phase(uvm_phase phase);
    link_frame cur = null;
    bit dl_last = 1;
    forever begin
      @(posedge vif.clk);
      dl_prev = dl_last; dl_last = (dl_vif == null) ? 1'b1 : dl_vif.dl_up;
      if (!vif.rst_n) begin cur = null; continue; end
      if (vif.valid) begin
        if (vif.dllp) begin
          link_frame f = link_frame::type_id::create("dllp");
          f.dir = dir; f.is_in = tap_is_in; f.start_cycle = cyc_now(); f.end_cycle = cyc_now(); f.beats = 1;
          for (int i = 0; i < 6; i++) f.bytes.push_back(vif.data[8*i +: 8]);
          if (!(vif.sop && vif.eop && vif.keep == 16'h003F) && tap_is_in)
            `uvm_error("LK-2", $sformatf("%s: DLLP framing wrong sop=%0b eop=%0b keep=%h", get_full_name(), vif.sop, vif.eop, vif.keep))
          fill_dllp(f); n_dllp++; ap.write(f);
          cur = null;
        end else begin
          int nb = 0;
          if (vif.sop) begin
            if (cur != null && tap_is_in) `uvm_error("LK-1", $sformatf("%s: sop inside a TLP frame", get_full_name()))
            cur = link_frame::type_id::create("tlp");
            cur.dir = dir; cur.is_in = tap_is_in; cur.start_cycle = cyc_now();
          end else if (cur == null) begin
            if (tap_is_in) `uvm_error("LK-1", $sformatf("%s: TLP beat without sop", get_full_name()))
            n_bad_framing++;
          end
          if (cur != null) begin
            for (int i = 0; i < 16; i++) if (vif.keep[i]) begin cur.bytes.push_back(vif.data[8*i +: 8]); nb++; end
            if (!vif.eop && vif.keep != 16'hFFFF && tap_is_in) `uvm_error("LK-3", $sformatf("%s: non-final beat with keep=%h", get_full_name(), vif.keep))
            cur.beats++;
            if (vif.eop) begin
              cur.end_cycle = cyc_now();
              fill_tlp(cur); n_tlp++; ap.write(cur); cur = null;
            end
          end
        end
      end else if (cur != null) begin
        if (dl_vif != null && (!dl_vif.dl_up || !dl_prev)) n_aborted++;   // LM-8: the link went down mid-frame
        else if (tap_is_in) `uvm_error("LK-1", $sformatf("%s: gap inside a TLP frame", get_full_name()))
        cur = null;
      end
    end
  endtask
endclass

// ---------------------------------------------------------------- link fault-injection agent
typedef uvm_sequencer #(fi_cmd) fi_sequencer;

class fi_driver extends uvm_driver #(fi_cmd);
  `uvm_component_utils(fi_driver)
  virtual fi_if vif;
  function new(string n, uvm_component p); super.new(n, p); endfunction
  function void build_phase(uvm_phase phase);
    if (!uvm_config_db#(virtual fi_if)::get(this, "", "vif", vif)) `uvm_fatal("NOVIF", "fi_driver: vif not set")
  endfunction
  task run_phase(uvm_phase phase);
    vif.fi <= '0;
    forever begin
      seq_item_port.get_next_item(req);
      @(posedge vif.clk);
      vif.fi <= req.cmd;
      seq_item_port.item_done();
    end
  endtask
endclass

class fi_monitor extends uvm_monitor;
  `uvm_component_utils(fi_monitor)
  virtual fi_if vif;
  uvm_analysis_port #(fi_evt) ap;
  int dir;
  int unsigned n_evt;
  int unsigned by_action [16];
  function new(string n, uvm_component p); super.new(n, p); endfunction
  function void build_phase(uvm_phase phase);
    ap = new("ap", this);
    if (!uvm_config_db#(virtual fi_if)::get(this, "", "vif", vif)) `uvm_fatal("NOVIF", "fi_monitor: vif not set")
    if (!uvm_config_db#(int)::get(this, "", "dir", dir)) dir = 0;
  endfunction
  task run_phase(uvm_phase phase);
    forever begin
      @(posedge vif.clk);
      if (vif.rst_n && vif.evt_valid) begin
        fi_evt e = fi_evt::type_id::create("fi_evt");
        e.dir = dir; e.action = vif.evt_action; e.seq = vif.evt_seq; e.dllp = vif.evt_dllp; e.idx = vif.evt_idx; e.t = cyc_now();
        n_evt++; by_action[e.action]++;
        ap.write(e);
      end
    end
  endtask
endclass

class fi_agent extends uvm_agent;
  `uvm_component_utils(fi_agent)
  fi_sequencer sqr; fi_driver drv; fi_monitor mon;
  function new(string n, uvm_component p); super.new(n, p); endfunction
  function void build_phase(uvm_phase phase);
    sqr = fi_sequencer::type_id::create("sqr", this);
    drv = fi_driver::type_id::create("drv", this);
    mon = fi_monitor::type_id::create("mon", this);
  endfunction
  function void connect_phase(uvm_phase phase);
    drv.seq_item_port.connect(sqr.seq_item_export);
  endfunction
endclass

// ---------------------------------------------------------------- dl_up status monitor
class status_monitor extends uvm_monitor;
  `uvm_component_utils(status_monitor)
  virtual csr_if vif [2];
  uvm_analysis_port #(dl_event) ap;
  bit up [2];
  bit fatal_seen [2];
  function new(string n, uvm_component p); super.new(n, p); endfunction
  function void build_phase(uvm_phase phase);
    ap = new("ap", this);
    if (!uvm_config_db#(virtual csr_if)::get(this, "", "vif0", vif[0])) `uvm_fatal("NOVIF", "status_monitor vif0")
    if (!uvm_config_db#(virtual csr_if)::get(this, "", "vif1", vif[1])) `uvm_fatal("NOVIF", "status_monitor vif1")
  endfunction
  task run_phase(uvm_phase phase);
    forever begin
      @(posedge vif[0].clk);
      for (int n = 0; n < 2; n++) begin
        if (vif[n].rst_n && vif[n].dl_up !== up[n]) begin
          dl_event e = dl_event::type_id::create("dl_ev");
          up[n] = vif[n].dl_up; e.node = n; e.up = up[n]; e.t = cyc_now();
          `uvm_info("DL", $sformatf("node%0d dl_up -> %0b @%0d", n, e.up, e.t), UVM_MEDIUM)
          ap.write(e);
        end
        if (vif[n].fatal_err) fatal_seen[n] = 1;
      end
    end
  endtask
endclass
