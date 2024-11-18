//Author: Lars Fischer
module fifo #(
    parameter DATA_WIDTH = 8,       // Width of the data bus
    parameter DEPTH = 8            // Depth of the FIFO
)(
    input clk,                 // Clock signal
    input rst_n,                 // Synchronous reset (active high)
    
    // Write Interface
    input wr_en,               // Write enable
    input [DATA_WIDTH-1:0] wr_data, // Data to write
    output wr_ready,           // FIFO ready to accept write
    
    // Read Interface
    input rd_en,               // Read enable
    output [DATA_WIDTH-1:0] rd_data, // Data output
    output rd_valid            // Data is valid
);

    // Calculate the address width based on DEPTH
    localparam ADDR_WIDTH = $clog2(DEPTH);
    
    // Memory array to store FIFO data
    logic [DATA_WIDTH-1:0] fifo_mem [0:DEPTH-1];
    
    // Write and read pointers
    logic [ADDR_WIDTH-1:0] wr_ptr;
    logic [ADDR_WIDTH-1:0] rd_ptr;
    logic [ADDR_WIDTH-1:0] counter;
    
    // FIFO status flags
    logic full = 0;
    logic empty = 1;
    
    // Next state logic for full and empty flags
    always @(posedge clk) begin
        if (~rst_n) begin
            wr_ptr <= 0;
            rd_ptr <= 0;
            full <= 0;
            empty <= 1;
	    counter <= 0;
        end else begin
            // Handle write operations
            
	    if (wr_en && wr_ready) begin
                fifo_mem[wr_ptr] <= wr_data;
                wr_ptr <= wr_ptr + 1;
            end
            
            // Handle read operations
            if (rd_en && rd_valid) begin
                rd_ptr <= rd_ptr + 1;

            end
            
            // Update counter and flags
            if (wr_en && wr_ready && !(rd_en && rd_valid)) begin
                if (wr_ptr + 1'b1  == rd_ptr)
                    full <= 1;
                empty <= 0;
		counter <= counter + 1;
            end else if (!(wr_en && wr_ready) && (rd_en && rd_valid)) begin
                if (rd_ptr + 1'b1 == wr_ptr)
                    empty <= 1;
	    	full <= 0;
		counter <= counter - 1;
	    end
        end
    end
    
    // Assign FIFO status signals
    assign wr_ready = ~full;
    assign rd_valid = ~empty;
    
    // Data output logic
    assign rd_data = fifo_mem[rd_ptr];

    // Counter logic -- not relevant for the design, only used for
    // verification



NO_WRITE_TO_FULL_FIFO:assert property(@(posedge clk) disable iff (~rst_n) 
									full |-> !(wr_ready && wr_en));
NO_WRITE_TO_FULL_FIFO2:assert property(@(posedge clk) disable iff (~rst_n) 
									(counter == DEPTH) |-> full);
NO_READ_FROM_EMPTY_FIFO:assert property(@(posedge clk) disable iff (~rst_n) 
									empty |-> !(rd_valid && rd_en));
NO_READ_FROM_EMPTY_FIFO2:assert property(@(posedge clk) disable iff (~rst_n) 
									(counter == 0) |-> empty);
endmodule
