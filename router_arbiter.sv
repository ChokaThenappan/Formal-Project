// 4 inputs to 1 output router arbiter
//
// There is no delay from request to grant.
// The abriter assumes that the request remains stable while the entire
// packet is forwarded. Hence, priority is updated whenever a tail flit
// is forwarded. Grant is locked between a head flit and the corresponding
// tail flit.
//
// Interface
//
// * Inputs
// - clk: clock.
// - rst: active-high reset.
// - request: each bit should be set to 1 if there is a valid flit coming from the corresponding
//   input port that needs to be routed to the output port arbitrated by this module.
// - forwarding_head: set to 1 to indicate the head flit of a new packet is being routed this cycle.
//   The current grant gets locked until the tail flit is forwarded (wormhole routing)
// - forwarding_tail: set to 1 to indicate the tail flit of a packet is being routed this cycle.
//   Priority is updated and grant is unlocked.
//
// * Outputs
// - grant: one-hot or zero. When grant[i] is set, request[i] is granted and the packet from the
//   corresponding input port i can be routed.
//   and the packet from the input
// - grant_valid: this flag indicates whether the current grant output is valid. When at least one
//   request bit is set, the arbiter grants the next higher priority request with zero-cycles delay,
//   unless grant is locked.
//

module router_arbiter
  (
   input  logic clk,
   input  logic rst,
   input  logic [3:0] request,
   input  logic forwarding_head,
   input  logic forwarding_tail,
   output logic [3:0] grant,
   output logic grant_valid
   );

  logic grant_locked;

  // Lock current grant for flit between head and tail, tail included
  always_ff @(posedge clk) begin
    if (rst) begin
      grant_locked <= 1'b0;
    end else begin
      if (forwarding_tail) begin
        grant_locked <= 1'b0;
      end else if (forwarding_head) begin
        grant_locked <= 1'b1;
      end
    end
  end

  assign grant_valid = |request & ~grant_locked;

  // Update priority
  typedef logic [3:0][3:0] priority_t;
  priority_t priority_mask, priority_mask_next;
  priority_t grant_stage1;
  logic [3:0][1:0] grant_stage2;

  // Higher priority is given to request[0] at reset
  localparam priority_t InitialPriority = { 4'b0000,   // request[3]
                                            4'b1000,   // request[2]
                                            4'b1100,   // request[1]
					    4'b1110 }; // request[0]

  always_ff @(posedge clk) begin
    if (rst) begin
      priority_mask <= InitialPriority;
    end else if (forwarding_head) begin
      priority_mask <= priority_mask_next;
    end
  end

  always_comb begin
    priority_mask_next = priority_mask;

    unique case (grant)
      4'b0001 : begin
        priority_mask_next[0] = '0;
        priority_mask_next[1][0] = 1'b1;
        priority_mask_next[2][0] = 1'b1;
        priority_mask_next[3][0] = 1'b1;
      end
      4'b0010 : begin
        priority_mask_next[1] = '0;
        priority_mask_next[0][1] = 1'b1;
        priority_mask_next[2][1] = 1'b1;
        priority_mask_next[3][1] = 1'b1;
      end
      4'b0100 : begin
        priority_mask_next[2] = '0;
        priority_mask_next[0][2] = 1'b1;
        priority_mask_next[1][2] = 1'b1;
        priority_mask_next[3][2] = 1'b1;
      end
      4'b1000 : begin
        priority_mask_next[3] = '0;
        priority_mask_next[0][3] = 1'b1;
        priority_mask_next[1][3] = 1'b1;
        priority_mask_next[2][3] = 1'b1;
      end
      default begin
      end
    endcase
  end

  genvar g_i, g_j;
  for (g_i = 0; g_i < 4; g_i++) begin : gen_grant

    for (g_j = 0; g_j < 4; g_j++) begin : gen_grant_stage1
      assign grant_stage1[g_i][g_j] = request[g_j] & priority_mask[g_j][g_i];
    end

    for (g_j = 0; g_j < 2; g_j++) begin : gen_grant_stage2
      assign grant_stage2[g_i][g_j] = ~(grant_stage1[g_i][2*g_j] | grant_stage1[g_i][2*g_j + 1]);
    end

    assign grant[g_i] = &grant_stage2[g_i] & request[g_i];

  end  // gen_grant

  //
  // Assertions
  //

`ifndef SYNTHESIS
// pragma coverage off
//VCS coverage off

  a_grant_onehot: assert property (@(posedge clk) disable iff(rst) $onehot0(grant))
    else $error("Fail: a_grant_onehot");	// Proven in JG

///////////////////////////////////////////////// Assume Properties - Start /////////////////////////////////////////////////

  // Assume Head Flit forwarded first and then Tail Flit
  a_head_tail_sequence: assume property (@(posedge clk) disable iff(rst)
    	(forwarding_tail |-> $past(forwarding_head)));

  a_tail_head_sequence: assume property (@(posedge clk) disable iff(rst)
    	(forwarding_head |-> $past(forwarding_tail)));

  // Assume request remains stable until tail flit
  a_stable_request: assume property (@(posedge clk) disable iff(rst)
    	(|request && !forwarding_tail |=> $stable(request)));

  // Assume head and tail flits cannot be forwarded in same cycle
  a_head_tail_exclusive: assume property (@(posedge clk) disable iff(rst)
    	!(forwarding_head && forwarding_tail));

  // Assume request must be valid when forwarding head or tail
  a_valid_flit: assume property (@(posedge clk) disable iff(rst)
    	((forwarding_head || forwarding_tail) |-> |request));

  // Assume no new request can arrive while grant is locked
  a_no_new_request: assume property (@(posedge clk) disable iff(rst)
    	(grant_locked |-> !($rose(request))));

  // Assume foward_head high for one cycle
  a_forward_head: assume property (@(posedge clk) disable iff(rst)
    	(forwarding_head |=> !(forwarding_head)));

  // Assume foward_tail high for one cycle
  a_forward_tail: assume property (@(posedge clk) disable iff(rst)
    	(forwarding_tail |=> !(forwarding_tail)));

///////////////////////////////////////////////// Assume Properties - End /////////////////////////////////////////////////


///////////////////////////////////////////////// Assert Properties - Start /////////////////////////////////////////////////

  //1
  a_no_request_no_grant: assert property (@(posedge clk) disable iff(rst) 
	(request == 4'b000) |-> (grant == 4'b0000)) 
    else $error("Fail: a_no_request_no_grant");	// Proven in JG
  
  //2
  a_grant_lock_zero: assert property (@(posedge clk) disable iff(rst) 
	(grant_locked == 0) |-> ($onehot0(grant))) 
    else $error("Fail: a_grant_lock_zero");	// Proven in JG

  //3
  a_stable_priority: assert property (@(posedge clk) disable iff(rst)
	(grant_locked == 1) |=> $stable(priority_mask))
    else $error("Fail: a_stable_priority");	// Proven in JG

  //4
  a_grant_valid: assert property (@(posedge clk) disable iff(rst)
	(!grant_locked && |request |-> grant_valid))
    else $error("Fail: a_grant_valid");		// Proven in JG

  //5
  a_grant_unlocked: assert property (@(posedge clk) disable iff(rst)
	(!grant_locked && forwarding_head |=> grant_locked))
    else $error("Fail: a_grant_unlocked");	// Proven in JG

  //6
  a_grant_locked: assert property (@(posedge clk) disable iff(rst)
	(grant_locked && forwarding_tail |=> !grant_locked))
    else $error("Fail: a_grant_locked");	//Proven in JG

  //7
  genvar g_i;
  for (g_i = 0; g_i < 4; g_i++) begin
  a_priority_changed: assert property (@(posedge clk) disable iff(rst)
	(((forwarding_head == 1) && (grant[g_i] && (priority_mask[g_i] !='0))) |=> $changed(priority_mask)))
    else $error("Fail: a_priority_changed");	//Proven in JG
  end
  //8
  a_one_input_one_output: assert property (@(posedge clk) disable iff(rst)
	($countones(grant) <= 1))
    else $error("Fail: a_one_input_one_output");	//Proven in JG

///////////////////////////////////////////////// Assert Properties - Start /////////////////////////////////////////////////

 // pragma coverage on
//VCS coverage on
`endif // ~SYNTHESIS

endmodule
