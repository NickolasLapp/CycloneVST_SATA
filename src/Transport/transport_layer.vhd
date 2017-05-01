library ieee;                   --! Use standard library.
use ieee.std_logic_1164.all;    --! Use standard logic elements
use ieee.numeric_std.all;       --! Use numeric standard

--custom packages
use work.sata_defines.all;
use work.transport_layer_pkg.all;
----------------------------------------------------------------------------
--Status to user Truth Table (All active high logic):
-- Status(0) == Device Ready
-- Status(1) == Write Ready
-- Status(2) == Send Read Ready
-- Status(3) == Retrieve Read Ready
-- Status(4) == Write Error
-- Status(5) == Read Error

--Command truth table
-- 000 == Do Nothing
-- X01 == Send Write     (Command to write data at specified address in SSD)
-- X10 == Send Read      (Command to retrieve data at specified address from SSD)
-- 1XX == Retrieve Read  (Command to read value from Rx buffer)
----------------------------------------------------------------------------
entity transport_layer is
   port(
        --Interface with Application Layer
        rst_n           :   in std_logic;
        clk         :   in std_logic;

        data_from_user      :   in std_logic_vector(DATA_WIDTH - 1 downto 0);
        address_from_user   :   in std_logic_vector(DATA_WIDTH - 1 downto 0);

        user_command            :   in std_logic_vector(2 downto 0);
        clear_errors            :   in std_logic;
        status_to_user          :   out std_logic_vector(5 downto 0);

        data_to_user       :   out std_logic_vector(DATA_WIDTH - 1 downto 0);
        address_to_user    :   out std_logic_vector(DATA_WIDTH - 1 downto 0);

        --Interface with Link Layer
        status_to_link :    out std_logic_vector(7 downto 0);
        status_from_link     :   in std_logic_vector(7 downto 0);
        data_to_link     :   out std_logic_vector(DATA_WIDTH - 1 downto 0);
        data_from_link      :   in std_logic_vector(DATA_WIDTH - 1 downto 0));

end transport_layer;

architecture transport_layer_arch of transport_layer is

    signal current_state, next_state : State_Type; --Signals for state machines

    --======================================================================================
    signal lba, read_lba : std_logic_vector(47 downto 0);   --address to write to / read from
    signal error_address : std_logic_vector(DATA_WIDTH - 1 downto 0); --If there is an error, updates with address of read or write
    signal last_read_address : data_width_array_type;   --array of two 32 bit address vectors, used to track read addresses
    signal read_error, write_error : std_logic; --error flags, used to update the error bits in the status to user
    signal tx_fis_array, rx_fis_array   :   register_fis_array_type; -- signals to hold host to device register FIS contents

--used in combination with the phyrdy_n signal from the link layer to indicate to the user when the controller is read to receive a command
    signal transport_ready : std_logic;

    --status signals from the link layer
    signal link_rdy, pause, data_from_link_valid, link_error : std_logic;

    --to link interface status signals
    signal tx_to_link_request : std_logic; --high when transport layer has data to send to link
    signal rx_from_link_ready : std_logic; --high when transport layer is waiting for data from link
    signal link_fis_type : std_logic_vector(7 downto 0);  --used to get the FIS type from the link layer data when a fis is received

    --constants to index into status to user vector
    constant DEVICE_READY_INDEX         : integer := 0;
    constant WRITE_VALID_INDEX          : integer := 1;
    constant SEND_READ_VALID_INDEX      : integer := 2;
    constant RETRIEVE_READ_VALID_INDEX  : integer := 3;
    constant WRITE_ERROR_INDEX          : integer := 4;
    constant READ_ERROR_INDEX           : integer := 5;

    constant STATUS_ERR : integer := 16;--index of error bit in first word of register device to host FIS
    constant STATUS_DF : integer := 21; --index of device fault bit in first word of register device to host FIS

    --======================================================================================
    --Buffer signals
    signal tx_buf_data_in, tx_buf_data_out : data_width_array_type; --data in and out signals for the two tx data buffers
    signal tx_wren : std_logic_vector(1 downto 0);                  --write enable signals for the two tx data buffers
    signal rx_buf_data_in, rx_buf_data_out : data_width_array_type; --data in and out signals for the two rx data buffers
    signal rx_wren : std_logic_vector(1 downto 0);                  --write enable signals for the two rx data buffers
    signal tx_buffer_full, rx_buffer_full, tx_buffer_empty, rx_buffer_empty   : std_logic_vector(1 downto 0);
    signal tx0_locked, tx1_locked, rx0_locked, rx1_locked : std_logic; -- signal to allow state machine to take control of buffers
    signal tx_index : integer range 0 to 1; -- signal to pick which buffer will be used by the state machine during a write
    signal rx_index : integer range 0 to 1; -- signal to pick which buffer will be used by the state machine during a read

    signal tx_write_addr, rx_write_addr : data_width_array_type;                    --write addresses for the tx and rx buffers
    signal tx_read_addr, rx_read_addr : std_logic_vector (DATA_WIDTH - 1 downto 0); --read addresses for the tx and rx buffers

    signal tx_write_ptr, tx_read_ptr : integer range 0 to BUFFER_DEPTH + 1; --integer to convert to tx buffer read/write address
    signal rx_write_ptr, rx_read_ptr : integer range 0 to BUFFER_DEPTH + 1; --integer to convert to rx buffer read/write address

    signal rx_buffer_read_select : integer range 0 to 1;    --signal to select which rx buffer to use during a retrieve read

    signal tx_buffer_data_valid : std_logic; --signal used to control reading from the tx buffer when write data is sent to the link layer
    signal rx_buffer_data_valid : std_logic; --signal used to control reading from the rx buffer during a retrieve read
    --======================================================================================

    signal s_data_to_link : std_logic_vector (DATA_WIDTH - 1 downto 0); --data_to_link values from the output logic of the state machine

    component buffer_2prt_ram is    --2prt ram component created using the altera quartus ip builder
        port
        (
            clock       : in std_logic  := '1';
            data        : in std_logic_vector (31 downto 0);
            rdaddress       : in std_logic_vector (10 downto 0);
            wraddress       : in std_logic_vector (10 downto 0);
            wren        : in std_logic  := '0';
            q       : out std_logic_vector (31 downto 0)
        );
    end component buffer_2prt_ram;

begin
    --Using four buffers total, 2 for writes (tx buffers) and two for reads (rx buffers).
    tx_buffer_0 : buffer_2prt_ram
        port map(
                    clock => clk,
                    data => tx_buf_data_in(0),
                    rdaddress => tx_read_addr(10 downto 0),--Quartus IP only defines 11 address bits because RAM depth is fixed
                    wraddress => tx_write_addr(0)(10 downto 0),--Quartus IP only defines 11 address bits because RAM depth is fixed
                    wren => tx_wren(0),
                    q => tx_buf_data_out(0)
            );
    tx_buffer_1 : buffer_2prt_ram
        port map(
                    clock => clk,
                    data => tx_buf_data_in(1),
                    rdaddress => tx_read_addr(10 downto 0),--Quartus IP only defines 11 address bits because RAM depth is fixed
                    wraddress => tx_write_addr(1)(10 downto 0),--Quartus IP only defines 11 address bits because RAM depth is fixed
                    wren => tx_wren(1),
                    q => tx_buf_data_out(1)
            );
    rx_buffer_0 : buffer_2prt_ram
        port map(
                    clock => clk,
                    data => rx_buf_data_in(0),
                    rdaddress => rx_read_addr(10 downto 0),--Quartus IP only defines 11 address bits because RAM depth is fixed
                    wraddress => rx_write_addr(0)(10 downto 0),--Quartus IP only defines 11 address bits because RAM depth is fixed
                    wren => rx_wren(0),
                    q => rx_buf_data_out(0)
            );
    rx_buffer_1 : buffer_2prt_ram
        port map(
                    clock => clk,
                    data => rx_buf_data_in(1),
                    rdaddress => rx_read_addr(10 downto 0),--Quartus IP only defines 11 address bits because RAM depth is fixed
                    wraddress => rx_write_addr(1)(10 downto 0),--Quartus IP only defines 11 address bits because RAM depth is fixed
                    wren => rx_wren(1),
                    q => rx_buf_data_out(1)
            );

--=================================================================================================================
--Transport Layer Finite State Machine
--=================================================================================================================
    transport_state_memory  :   process(clk, rst_n)
      begin
        if (rst_n = '0') then
            current_state <= transport_reset;
        elsif (rising_edge(clk)) then
            if (pause = '0') then   --if the link asserts pause the transport state should not update
                current_state <= next_state;
            end if;
        end if;
    end process;

    transport_next_state_logic: process (current_state, status_from_link, link_rdy, data_from_link,link_fis_type, user_command,rst_n,pause,
                                                     tx_index, rx_index, tx_buffer_full, rx_buffer_full, tx_read_ptr, data_from_link_valid,
                                                                                                        read_error, write_error, link_error)
      begin
        case (current_state) is
        ----------------------------------------------- -----------------------------------------------
            -- Idle SM states (top level)
        ----------------------------------------------- -----------------------------------------------
            when transport_reset =>
                if (rst_n = '0') then
                    next_state <= transport_reset;
                else
                    next_state <= transport_init_start;
                end if;
            when transport_init_start =>    --After reset, wait for device to send initial status update (A register device to host FIS)
                if (data_from_link(7 downto 0) = REG_DEVICE_TO_HOST) then
                    next_state <= transport_init_end;
                else
                    next_state <= transport_init_start;
                end if;
            when transport_init_end =>  --after receiving states, wait in this state until the end of the FIS
                if (link_error = '1') then
                    --error occurred. Loop back to init_start, because need status to come through for device to work
                    next_state <= transport_init_start;
                elsif (data_from_link_valid = '1') then
                    next_state <= transport_init_end; --link layer is still transmitting end of the FIS
                else
                    next_state <= identify_device_0; --link layer finished transmitting initial status FIS, move to identify device
                end if;
            when identify_device_0    =>    --after receiving initial state, send identify device FIS as part of required initialization process
                if (link_rdy = '1' and pause = '0') then
                    next_state <= identify_device_1; --link layer has accepted first data word, move to next state
                else
                    next_state <= identify_device_0;
                end if;
            when identify_device_1    =>
                if (link_rdy = '1' and pause = '0') then
                    next_state <= identify_device_2; --link layer has accepted second data word, move to next state
                else
                    next_state <= identify_device_1;
                end if;
            when identify_device_2    =>
                if (link_rdy = '1' and pause = '0') then
                    next_state <= identify_device_3; --link layer has accepted third data word, move to next state
                else
                    next_state <= identify_device_2;
                end if;
            when identify_device_3    =>
                if (link_rdy = '1' and pause = '0') then
                    next_state <= identify_device_4; --link layer has accepted fourth data word, move to next state
                else
                    next_state <= identify_device_3;
                end if;
            when identify_device_4    =>
                if (link_rdy = '1' and pause = '0') then --link layer has accepted fifth and final data word, move to next state
                    next_state <= rx_pio_setup;
                else
                    next_state <= identify_device_4;
                end if;
            when rx_pio_setup =>    --wait in this state until the PIO setup fis has been received
                if(link_error = '1') then
                    --device did not correctly receive identify_device fis, must retransmit to ensure valid operation
                    next_state <= identify_device_0;
                elsif (link_fis_type = PIO_SETUP_FIS) then
                    next_state <= rx_identify_packet; --received the PIO Setup FIS, a protocol command that comes before the identify device packet
                else
                    next_state <= rx_pio_setup;
                end if;
            when rx_identify_packet => --wait in this state until start of identify device data fis is being transmitted
                if (link_fis_type = DATA_FIS) then
                --currently not processing the identify device packet, because this project is only targeting a known SSD
                    next_state <= wait_for_fis_end;
                else
                    next_state <= rx_identify_packet;
                end if;
            when wait_for_fis_end =>
            --this state is used to receive the end of FISs from the device when it is known that no more useful information will be transmitted
                if (data_from_link_valid = '1') then
                    next_state <= wait_for_fis_end;
                elsif (link_error = '1' or read_error = '1' or write_error = '1') then
                    --if any of the error bits have been set by the previous state, go to the report error state to transmit to the user
                    next_state <= report_error;
                else
                    next_state <= transport_idle; --otherwise return to the idle state
                end if;
            when transport_idle =>  --wait in this state until one of the write buffers fills up, or the user has sent a read command
                if (tx_buffer_full(0) = '1') then
                    next_state <= dma_write_idle; --first tx buffer is full
                elsif (tx_buffer_full(1) = '1') then
                    next_state <= dma_write_idle;   --second tx buffer is full
                elsif (user_command(1 downto 0) = "10") then
                    next_state <= dma_read_idle;    --user is sending a read
                else
                    next_state <= transport_idle;
                end if;
--========================================================================================
                -- DMA Write EXT SM states
            when dma_write_idle     =>  --register host to device FIS for the write command is built in this state
                next_state <= dma_write_reg_fis_0;
            --Therer are five states to transmit complete register host to device command, corresponding to 5-32 bit words
            when dma_write_reg_fis_0    =>  --wait in this state until the link layer is ready to receive data from transport layer
                if (link_rdy = '1' and pause = '0') then
                    next_state <= dma_write_reg_fis_1; --link has received first dword
                else
                    next_state <= dma_write_reg_fis_0;
                end if;
            when dma_write_reg_fis_1    =>
                if (link_rdy = '1' and pause = '0') then
                    next_state <= dma_write_reg_fis_2; --link has received second dword
                else
                    next_state <= dma_write_reg_fis_1;
                end if;
            when dma_write_reg_fis_2    =>
                if (link_rdy = '1' and pause = '0') then
                    next_state <= dma_write_reg_fis_3; --link has received third dword
                else
                    next_state <= dma_write_reg_fis_2;
                end if;
            when dma_write_reg_fis_3    =>
                if (link_rdy = '1' and pause = '0') then
                    next_state <= dma_write_reg_fis_4; --link has received fourth dword
                else
                    next_state <= dma_write_reg_fis_3;
                end if;
            when dma_write_reg_fis_4    =>
                if (link_rdy = '1' and pause = '0') then
                    next_state <= dma_write_chk_activate; --link has received fifth and final dword
                else
                    next_state <= dma_write_reg_fis_4;
                end if;
            --Device will send back an activate command after the write command has been sent if the command is valid
            when dma_write_chk_activate =>  --wait in this state until an activate is received from the device
                if (link_fis_type = DMA_ACTIVATE_FIS) then
                    next_state <= dma_write_data_fis;   --command was valid, device is ready to accept data
                elsif ((data_from_link (7 downto 0) = REG_DEVICE_TO_HOST and
                    (data_from_link(STATUS_ERR) = '1' or data_from_link(STATUS_DF) = '1')) or link_error = '1') then
                    next_state <= wait_for_fis_end; --command error, device returned an error status instead of the activate fis
                else
                    next_state <= dma_write_chk_activate; --still waiting for activate
                end if;
            when dma_write_data_fis => --wait here until link layer is ready to accept write data
                if (pause = '0' and link_rdy = '1') then
                    next_state <= dma_write_data_frame; --link is ready for remaining write data
                else
                    next_state <= dma_write_data_fis;
                end if;
            when dma_write_data_frame   =>
                if (tx_read_ptr < BUFFER_DEPTH and link_rdy = '1') then
                    --have not transmitted all of the data in the tx buffer and link layer is still ready to receive data
                    next_state <= dma_write_data_frame;
                else
                    next_state <= dma_write_chk_status; --finished sending data
                end if;
            when dma_write_chk_status   => --wait for device to return a state indicating command success or failure
                if (link_fis_type = REG_DEVICE_TO_HOST) then
                    next_state <= wait_for_fis_end; --received first word of status fis, move to fis end state until link is done transmitting
                else
                    next_state <= dma_write_chk_status;
                end if;
--========================================================================================
            -- DMA Read EXT SM states
            when dma_read_idle      =>  --create register host to device FIS with read command in this state
                    next_state <= dma_read_reg_fis_0;
            when dma_read_reg_fis_0 =>  --wait here until link has received first dword of read command
                if (link_rdy = '1' and pause = '0') then
                    next_state <= dma_read_reg_fis_1; --link has received first dword of register host to device FIS
                else
                    next_state <= dma_read_reg_fis_0;
                end if;
            when dma_read_reg_fis_1 =>
                if (link_rdy = '1' and pause = '0') then
                    next_state <= dma_read_reg_fis_2; --link has received second dword
                else
                    next_state <= dma_read_reg_fis_1;
                end if;
            when dma_read_reg_fis_2 =>
                if (link_rdy = '1' and pause = '0') then
                    next_state <= dma_read_reg_fis_3; --link has received third dword
                else
                    next_state <= dma_read_reg_fis_2;
                end if;
            when dma_read_reg_fis_3 =>
                if (link_rdy = '1' and pause = '0') then
                    next_state <= dma_read_reg_fis_4; --link has received fourth dword
                else
                    next_state <= dma_read_reg_fis_3;
                end if;
            when dma_read_reg_fis_4 =>
                if (link_rdy = '1' and pause = '0') then
                    next_state <= dma_read_data_fis; --link has received fifth and final dword of register host to device FIS
                else
                    next_state <= dma_read_reg_fis_4;
                end if;
            when dma_read_data_fis  => --wait here until device begins sending data back from the read
                if (data_from_link(7 downto 0)= DATA_FIS) then
                    next_state <= dma_read_data_frame; --device has begun sending data
                elsif ((data_from_link (7 downto 0) = REG_DEVICE_TO_HOST and
                        (data_from_link(STATUS_ERR) = '1' or data_from_link(STATUS_DF) = '1')) or link_error = '1') then
                    --read command was invalid: this condition is possible if address is out of range, or there is a device fault or media error
                    next_state <= wait_for_fis_end;
                else
                    next_state <= dma_read_data_fis; --device has not yet begun to send data
                end if;
            when dma_read_data_frame    => --stay here while the data buffer is not full and the input data is still valid
                if (rx_buffer_full(rx_index) = '0' and data_from_link_valid = '1') then
                    next_state <= dma_read_data_frame; --selected rx buffer is not full and data from link is still valid
                else
                    next_state <= dma_read_chk_status; --selected rx buffer is full
                end if;
            when dma_read_chk_status    =>  --wait for device to return a state indicating command success or failure
                if (link_fis_type = REG_DEVICE_TO_HOST) then
                    next_state <= wait_for_fis_end; --received first word of status fis, move to fis end state until link is done transmitting
                else
                    next_state <= dma_read_chk_status;
                end if;
            when report_error => --if there was an error, state machine will transition here until user has cleared the error
                if (read_error = '1' or write_error = '1') then
                    next_state <= report_error; --user has not yet cleared errors
                else
                    next_state <= transport_idle;   --user cleared errors, go back to idle
                end if;
--=======================================================================================
            when others =>  next_state <= transport_idle;
        end case;
    end process;
--=================================================================================================================
    transport_output_logic: process(clk,rst_n)
      begin
        if (rst_n = '0') then
            rx0_locked <= '0'; --locked signals are used to control data buffers
            rx1_locked <= '0';
            tx0_locked <= '0';
            tx1_locked <= '0';
            rx_from_link_ready <= '0'; --not ready to receive data
            tx_to_link_request <= '0'; --not ready to send data
            tx_buffer_empty <= "11";    --buffers start out empty
            rx_buffer_full <= "00";
            rx_wren <= "00";    --write enable signals for rx buffers
            tx_read_ptr <= 0; --pointer to read from the tx buffers during a write
            rx_write_ptr <= 0; --pointer to write to rx buffers during a read
            tx_index <= 0;
            transport_ready <= '0'; --this signal is only asserted at the end of the initialization process
            s_data_to_link <= (others => '1');
            tx_buffer_data_valid <= '0'; --data from tx buffers is not valid
            rx_buf_data_in <= (others => (others => '0'));
            rx_write_addr <= (others => (others => '0'));
            last_read_address <= (others => (others => '1')); --set addresses to all 0xF's after reset
            error_address <= (others => '1');
            read_error <= '0';
            write_error <= '0';
            read_lba <= (others => '0');
        elsif (rising_edge(clk)) then
            if (pause = '0') then
                case (current_state) is
                ----------------------------------------------- -----------------------------------------------
                    -- Idle SM states (top level)
                ----------------------------------------------- -----------------------------------------------
                    when transport_reset =>
                        rx0_locked <= '0'; --locked signals are used to control data buffers
                        rx1_locked <= '0';
                        tx0_locked <= '0';
                        tx1_locked <= '0';
                        rx_from_link_ready <= '0'; --not ready to receive data
                        tx_to_link_request <= '0'; --not ready to send data
                        tx_buffer_empty <= "11";    --buffers start out empty
                        rx_buffer_full <= "00";
                        rx_wren <= "00";    --write enable signals for rx buffers
                        tx_read_ptr <= 0; --pointer to read from the tx buffers during a write
                        rx_write_ptr <= 0; --pointer to write to rx buffers during a read
                        tx_index <= 0;
                        transport_ready <= '0'; --this signal is only asserted at the end of the initialization process
                        s_data_to_link <= (others => '1');
                        tx_buffer_data_valid <= '0'; --data from tx buffers is not valid
                        rx_buf_data_in <= (others => (others => '0'));
                        rx_write_addr <= (others => (others => '0'));
                        last_read_address <= (others => (others => '1')); --set addresses to all 0xF's after reset
                        error_address <= (others => '1');
                        read_error <= '0';
                        write_error <= '0';
                        read_lba <= (others => '0');
                    when transport_init_start =>
                        --wait for inital device status to be sent from link layer
                        rx_from_link_ready <= '1';
                        --build register host to device command to send identify device FIS
                        tx_fis_array(tx_index).fis_type <= REG_HOST_TO_DEVICE; --Set FIS type
                        tx_fis_array(tx_index).crrr_pm <= x"80"; --80 sets "c" bit, required
                        tx_fis_array(tx_index).command <= IDENTIFY_DEVICE; --Set command type
                        tx_fis_array(tx_index).features <= x"00"; --field not used for this command
                        tx_fis_array(tx_index).lba <= (others => '0'); --field not used for this command
                        tx_fis_array(tx_index).device <= x"E0"; --required
                        tx_fis_array(tx_index).features_ext <= x"00"; --field not used for this command
                        tx_fis_array(tx_index).lba_ext <= (others => '0'); --field not used for this command
                        tx_fis_array(tx_index).count <= (others => '0'); --on most drives 1 logical sector := 512 bytes
                        tx_fis_array(tx_index).icc <= x"00"; --field not used for this command
                        tx_fis_array(tx_index).control <= x"00"; --field not used for this command
                        tx_fis_array(tx_index).aux <= x"00000000"; --field not used for this command
                    when transport_init_end =>
                        s_data_to_link <= (others => '1'); --set data to all 0xF's, only useful for debug
                    when identify_device_0 =>
                        --request to transmit data to link, and assert the first dword of the register device to host fis to the link data port
                        rx_from_link_ready <= '0';
                        tx_to_link_request <= '1';
                        --see the SATA 3.1 protocol documentation for explanation of features of the register host to device FIS
                        s_data_to_link <= tx_fis_array(tx_index).features & tx_fis_array(tx_index).command &
                                       tx_fis_array(tx_index).crrr_pm & tx_fis_array(tx_index).fis_type;
                    when identify_device_1 =>
                        s_data_to_link <= tx_fis_array(tx_index).device & tx_fis_array(tx_index).lba;
                    when identify_device_2 =>
                        s_data_to_link <= tx_fis_array(tx_index).features_ext &
                                        tx_fis_array(tx_index).lba_ext;
                    when identify_device_3 =>
                        s_data_to_link <= tx_fis_array(tx_index).control & tx_fis_array(tx_index).icc &
                                        tx_fis_array(tx_index).count;
                    when identify_device_4 =>
                        s_data_to_link <= tx_fis_array(tx_index).aux;
                    when rx_pio_setup =>    --wait in this state until link has begun sending the pio_setup FIS from the device
                        tx_to_link_request <= '0';
                        rx_from_link_ready <= '1';
                    when rx_identify_packet =>  --wait in this state until link has begun sending data fis for identify device command
                        s_data_to_link <= (others => '0'); --assign value for debug
                    when wait_for_fis_end =>    --general state used for receiving the end of a FIS when it is known the data is not useful
                        tx_to_link_request <= '0';
                        rx_from_link_ready <= '1';
                    when transport_idle => --wait in idle until a tx buffer is full, or the user sends a read command
                        transport_ready <= '1'; --transport_layer is ready, initialization process has finished
                        rx_from_link_ready <= '0';  --not receiving data from link
                        tx_to_link_request <= '0';  --not transmitting data to link
                        rx_wren <= "00";
                        if (rx_buffer_empty(0) = '1') then rx_buffer_full(0) <= '0'; end if; --update rx buffer full flags in this state
                        if (rx_buffer_empty(1) = '1') then rx_buffer_full(1) <= '0'; end if;
                        if (tx_buffer_full(0) = '1') then --first tx buffer is full, lock it until it has been written to the device
                            tx0_locked <= '1'; --lock buffer zero
                            tx_buffer_empty(0) <= '0';
                            tx_index <= 0; --select buffer zero
                        elsif (tx_buffer_full(1) = '1') then --second tx buffer is full, lock it until it has been written to the device
                                tx1_locked <= '1'; --lock buffer one
                                tx_buffer_empty(1) <= '0';
                                tx_index <= 1; --select buffer one
                        elsif (user_command(1 downto 0) = "10") then
                            --user is sending a read
                            if (rx_buffer_empty(0) = '1') then --if first buffer is empty, select it and use to store the read data
                                rx_index <= 0; --select buffer zero
                                rx0_locked <= '1'; --lock buffer zero
                                read_lba <= x"0000" & address_from_user; --else, if second buffer is empty select it and use to store the read data
                            elsif (rx_buffer_empty(1) = '1') then
                                rx_index <= 1;  --select buffer one
                                rx1_locked <= '1'; --lock buffer one
                            end if;
                        end if;
            ----------------------------------------------- --------------------------------------
        --========================================================================================
                    -- DMA Write EXT SM states
                    when dma_write_idle     =>
                        --build register host to device DMA Write FIS
                        tx_read_ptr <= 0;
                        tx_fis_array(tx_index).fis_type <= REG_HOST_TO_DEVICE; --Set FIS type
                        tx_fis_array(tx_index).crrr_pm <= x"80"; --80 sets C bit
                        tx_fis_array(tx_index).command <= WRITE_DMA_EXT; --Set command type
                        tx_fis_array(tx_index).features <= x"00"; --Field not used for this command
                        tx_fis_array(tx_index).lba <= lba(23 downto 0); --set lba to first 24 bits of logical block address
                        tx_fis_array(tx_index).device <= x"E0";
                        tx_fis_array(tx_index).features_ext <= x"00"; --Field not used for this command
                        tx_fis_array(tx_index).lba_ext <= lba(47 downto 24); --set lba_ext to last 24 bits of logical block address
                        tx_fis_array(tx_index).count <= WRITE_SECTOR_COUNT; --on most drives 1 logical sector := 512 bytes
                        tx_fis_array(tx_index).icc <= x"00"; --Field not used for this command
                        tx_fis_array(tx_index).control <= x"00"; --Field not used for this command
                        tx_fis_array(tx_index).aux <= x"00000000"; --Field not used for this command
                    when dma_write_reg_fis_0    => --transmit first dword of command to link layer
                        tx_to_link_request <= '1';
                        s_data_to_link <= tx_fis_array(tx_index).features & tx_fis_array(tx_index).command &
                                       tx_fis_array(tx_index).crrr_pm & tx_fis_array(tx_index).fis_type;
                    when dma_write_reg_fis_1    => --transmit second dword
                        s_data_to_link <= tx_fis_array(tx_index).device & tx_fis_array(tx_index).lba;
                    when dma_write_reg_fis_2    => --transmit third dword
                        s_data_to_link <= tx_fis_array(tx_index).features_ext &
                                       tx_fis_array(tx_index).lba_ext;
                    when dma_write_reg_fis_3    => --transmit fourth dword
                        s_data_to_link <= tx_fis_array(tx_index).control & tx_fis_array(tx_index).icc &
                                       tx_fis_array(tx_index).count;
                    when dma_write_reg_fis_4    => --transmit fifth and final dword
                        s_data_to_link <= tx_fis_array(tx_index).aux;
                    when dma_write_chk_activate => --wait in this state until activate has been received indicating write command is valid
                        tx_to_link_request <= '0';
                        rx_from_link_ready <= '1';
                        if ((data_from_link (7 downto 0) = REG_DEVICE_TO_HOST and
                                (data_from_link(STATUS_ERR) = '1' or data_from_link(STATUS_DF) = '1')) or link_error = '1') then
                            error_address <= tx_fis_array(tx_index).lba_ext(7 downto 0) & tx_fis_array(tx_index).lba;
                            write_error <= '1';
                        end if;
                    when dma_write_data_fis => --transmit first dword of data FIS (0x00000046)
                        rx_from_link_ready <= '0';
                        tx_to_link_request <= '1';
                        s_data_to_link <=  x"000000" & DATA_FIS;
                    when dma_write_data_frame   => --transmit write data from the selected tx buffer
                        if (pause = '0') then --don't increment pointer if transport is paused
                            if (tx_read_ptr < BUFFER_DEPTH) then
                                --if pointer is less than buffer depth data is valid and pointer can be incremented
                                tx_read_ptr <= tx_read_ptr + 1;
                                tx_buffer_data_valid <= '1';
                            else
                                --If pointer is equal to buffer depth, all of the data has been transmitted
                                tx_buffer_data_valid <= '0';
                                tx_to_link_request <= '0';
                                tx_buffer_empty(tx_index) <= '1'; --selected buffer is now empty
                                if (tx_index = 0) then --unlock selected buffer
                                    tx0_locked <= '0';
                                else
                                    tx1_locked <= '0';
                                end if;
                            end if;
                        end if;
                    when dma_write_chk_status   => --wait for status to be return in register device to host FIS
                        rx_from_link_ready <= '1';
                        if (link_error = '1') then --link layer has indicated a link level error has occurred
                            error_address <= tx_fis_array(tx_index).lba_ext(7 downto 0) & tx_fis_array(tx_index).lba;
                            write_error <= '1';
                        elsif (data_from_link (7 downto 0) = REG_DEVICE_TO_HOST) then
                            --Status has been received
                            if (data_from_link(STATUS_ERR) = '1' or data_from_link(STATUS_DF) = '1') then
                                --Error bit or device fault bit is set in state, command may have failed
                                error_address <= tx_fis_array(tx_index).lba_ext(7 downto 0) & tx_fis_array(tx_index).lba;
                                write_error <= '1';
                            end if;
                        end if;
        --========================================================================================
                    -- DMA Read EXT SM states
                    when dma_read_idle      =>
                        rx_write_ptr <= 0;
                        --build register host to device DMA Read FIS
                        rx_fis_array(rx_index).fis_type <= REG_HOST_TO_DEVICE; --Set FIS type
                        rx_fis_array(rx_index).crrr_pm <= x"80"; --80 sets C bit, required for this command type
                        rx_fis_array(rx_index).command <= READ_DMA_EXT; --set command type
                        rx_fis_array(rx_index).features <= x"00"; --Field not used for this command
                        rx_fis_array(rx_index).lba <= read_lba(23 downto 0); --set lba to first 24 bits of logical block address for read
                        rx_fis_array(rx_index).device <= x"E0";
                        rx_fis_array(rx_index).features_ext <= x"00"; --Field not used for this command
                        rx_fis_array(rx_index).lba_ext <= read_lba(47 downto 24); --set lba_ext to last 24 bits of logical block address for read
                        rx_fis_array(rx_index).count <= WRITE_SECTOR_COUNT; --on most drives 1 logical sector := 512 bytes
                        rx_fis_array(rx_index).icc <= x"00"; --Field not used for this command
                        rx_fis_array(rx_index).control <= x"00"; --Field not used for this command
                        rx_fis_array(rx_index).aux <= x"00000000"; --Field not used for this command
                    when dma_read_reg_fis_0 => --transmit first dword of register host to device fis for read command
                        tx_to_link_request <= '1';
                        s_data_to_link <= rx_fis_array(rx_index).features & rx_fis_array(rx_index).command &
                                       rx_fis_array(rx_index).crrr_pm & rx_fis_array(rx_index).fis_type;
                    when dma_read_reg_fis_1 => --transmit second dword
                        s_data_to_link <= rx_fis_array(rx_index).device & rx_fis_array(rx_index).lba;
                    when dma_read_reg_fis_2 => --transmit third dword
                        s_data_to_link <= rx_fis_array(rx_index).features_ext &
                                       rx_fis_array(rx_index).lba_ext;
                    when dma_read_reg_fis_3 => --transmit fourth dword
                        s_data_to_link <= rx_fis_array(rx_index).control & rx_fis_array(rx_index).icc &
                                       rx_fis_array(rx_index).count;
                    when dma_read_reg_fis_4 => --transmit fifth and final dword of register host to device fis
                        s_data_to_link <= rx_fis_array(rx_index).aux;
                    when dma_read_data_fis  => --wait until first word of data fis has been received
                        tx_to_link_request <= '0';
                        rx_from_link_ready <= '1';
                        if ((data_from_link (7 downto 0) = REG_DEVICE_TO_HOST and
                                (data_from_link(STATUS_ERR) = '1' or data_from_link(STATUS_DF) = '1')) or link_error = '1') then
                            --Register device to host fis received instead of data fis, an error has occurred
                            error_address <= rx_fis_array(rx_index).lba_ext(7 downto 0) & rx_fis_array(rx_index).lba;
                            read_error <= '1';
                        end if;
                    when dma_read_data_frame    => --receive data from logical block address specified in read command
                        if (pause = '0') then --don't increment buffer pointer if transport is paused
                            if (data_from_link_valid = '1') then --make sure link is transmitting data
                                if (rx_write_ptr < BUFFER_DEPTH) then
                                    rx_wren(rx_index) <= '1'; --if the above conditions are met, assert write enable for selected rx buffer
                                else
                                    rx_wren(rx_index) <= '0';
                                end if;
                                --set write address of selected rx buffer
                                rx_write_addr(rx_index) <= std_logic_vector(to_unsigned(rx_write_ptr,DATA_WIDTH));
                                rx_buf_data_in(rx_index) <= data_from_link; --send data from the link to the selected rx buffer
                                if (rx_write_ptr < BUFFER_DEPTH - 1) then --make sure buffer write pointer stays within valid range
                                    rx_write_ptr <= rx_write_ptr + 1;
                                else
                                    rx_buffer_full(rx_index) <= '1'; --selected buffer is now full
                                    --unlock selected buffer because it is now full
                                    if (rx_index = 0) then
                                        rx0_locked <= '0';
                                    else
                                        rx1_locked <= '0';
                                    end if;
                                end if;
                            end if;
                        end if;
                    when dma_read_chk_status =>
                        --wait for the device to send a register device to host device fis indicating command completion status
                        rx_wren(rx_index) <= '0'; --make sure write enable is de-asserted
                        --record address for read so it can be sent back to user when a retrieve read is sent to the selected buffer
                        last_read_address(rx_index) <= rx_fis_array(rx_index).lba_ext(7 downto 0) & rx_fis_array(rx_index).lba;
                        rx_from_link_ready <= '1';
                        if (link_error = '1') then --link has indicated an error occurred, record error address
                            error_address <= rx_fis_array(rx_index).lba_ext(7 downto 0) & rx_fis_array(rx_index).lba;
                            read_error <= '1'; --error occurred during a read
                        elsif (data_from_link (7 downto 0) = REG_DEVICE_TO_HOST) then
                            --register host to device fis received
                            if (data_from_link(STATUS_ERR) = '1' or data_from_link(STATUS_DF) = '1') then
                                --Error bit or device fault bit is set in state, command may have failed, record error address
                                error_address <= rx_fis_array(rx_index).lba_ext(7 downto 0) & rx_fis_array(rx_index).lba;
                                read_error <= '1'; --error occurred during a read
                            end if;
                        end if;
                    when report_error => --stay in this state until user clears error,
                        if (clear_errors = '1') then --user has asserted clear_errors, set all errors to zero
                            read_error <= '0';
                            write_error <= '0';
                        end if;
        --========================================================================================
                    when others => -- state <= transport_idle;
                end case;
            end if;
        end if;
    end process;

--=================================================================================================================
--logic and processes to handle tx read RAM access
tx_buffer_reader : process (rst_n, pause, tx_read_ptr, tx_buffer_data_valid)
  begin
    if (rst_n = '0') then
        tx_read_addr <= (others => '0');
    elsif (tx_read_ptr < BUFFER_DEPTH and tx_buffer_data_valid = '1') then
        --tx_read_ptr is updated in output_logic process, this process converts it to a std_logic_vector
        --The same address is used for both tx buffers because they will never be read at the same time
        if (pause = '0') then
            tx_read_addr <= std_logic_vector(to_unsigned(tx_read_ptr,DATA_WIDTH));
        else
            --if transport was paused need to go back to the previous data value from the selected tx buffer
            tx_read_addr <= std_logic_vector(to_unsigned(tx_read_ptr-1,DATA_WIDTH));
        end if;
    else
        tx_read_addr <= (others => '0');
    end if;
end process;

switch_data_to_link : process(rst_n, pause, s_data_to_link, tx_buf_data_out,tx_buffer_data_valid, tx_index)
  --this process switches the data_to_link port between the s_data_to_link signal and the data out from the tx
  --buffers depending on which is desired at the current time. The tx_buffer_data_valid signal is used to control this.
  begin
    if (rst_n = '0') then
        data_to_link <= (others => '0');
    elsif (tx_buffer_data_valid = '1') then
        data_to_link <= tx_buf_data_out(tx_index); --data to link is output data from the selected tx buffer
    else
        data_to_link <= s_data_to_link; --data to link is the s_data_to_link signal updated in the output_logic process
    end if;
end process;
--=================================================================================================================
--Processes to control the flow of user data to/from the tx/rx buffers
--The dual-buffer system allows user data to be written to a buffer even when the Transport FSM is performing a command
 tx_buffer_control   : process(clk,rst_n)
    variable tx_w_buffer : integer range 0 to 1; --used to select which tx buffer user data is written to
    variable user_write_valid : std_logic; --internal signal for this process to ensure write is valid
      begin
        if (rst_n = '0') then
            tx_write_ptr <= 0;
            tx_w_buffer := 0;
            user_write_valid := '0';
            tx_buffer_full(0) <= '0';
            tx_buffer_full(1) <= '0';
            tx_wren <= "00";
            tx_write_addr <= (others => (others => '0'));
            tx_buf_data_in <= (others => (others => '0'));
            lba <= (others => '0');
        elsif (rising_edge(clk)) then

            if (tx_buffer_full(0) = '0' and tx0_locked = '0') then
                --first buffer can be written to, set appropriate write enable
                tx_w_buffer := 0;
                user_write_valid := '1';
                tx_wren(0) <= '1';
                tx_wren(1) <= '0';
            elsif (tx_buffer_full(1) = '0' and tx1_locked = '0') then
                --second buffer can be written to, set appropriate write enable
                tx_w_buffer := 1;
                user_write_valid := '1';
                tx_wren(0) <= '0';
                tx_wren(1) <= '1';
            else
                --both buffers are full or locked
                user_write_valid := '0';
                tx_wren <= "00";
            end if;

            if (tx_read_ptr > 0) then --if this is true, one of the buffers is being read to send the data to the device during a write
                --if either buffer has been locked, the data is being transmitted to the link layer and the buffer is no longer full
                if (tx0_locked = '1') then
                    tx_buffer_full(0) <= '0';
                elsif (tx1_locked = '1') then
                    tx_buffer_full(1) <= '0';
                end if;
            end if;

            if (user_command(1 downto 0) = "01" and user_write_valid = '1') then --user is sending data, and command is valid
                if (tx_write_ptr < BUFFER_DEPTH - 1) then --make sure pointer stays within valid range
                    tx_write_ptr <= tx_write_ptr + 1;
                else
                    tx_write_ptr <= 0;
                    tx_buffer_full(tx_w_buffer) <= '1'; --Pointer has reached BUFFER_DEPTH - 1, selected buffer is full
                end if;
                lba <= x"0000" &  address_from_user; --User required to keep address valid through the end of a write command
                tx_write_addr(tx_w_buffer) <= std_logic_vector(to_unsigned(tx_write_ptr,DATA_WIDTH)); --update tx buffer write address
                tx_buf_data_in(tx_w_buffer) <= data_from_user; --send user data to the selected tx buffer
            end if;
        end if;
    end process;

rx_buffer_control_reads : process(clk, rst_n)
    --this process is used to control the retrieve read commands
    variable user_rx_read_valid : std_logic; --internal signal to ensure retrieve read command is valid
      begin
        if (rst_n = '0') then
            rx_read_ptr <= 0;
            user_rx_read_valid := '0';
            rx_buffer_empty <= "11";--both buffers start empty after reset
            rx_buffer_data_valid <= '0';
            rx_buffer_read_select <= 0;
        elsif (rising_edge(clk)) then
            if (rx_write_ptr > 0) then --if this is true, one of the rx_buffers is being written to during a send read command
                --The locked buffer is the one being written to, the appropriate empty flag may now be cleared
                if (rx0_locked = '1') then
                    rx_buffer_empty(0) <= '0';
                elsif (rx1_locked = '1') then
                    rx_buffer_empty(1) <= '0';
                end if;
            end if;

            if (rx0_locked = '0' and rx_buffer_empty(0) = '0') then --if a buffer is not locked, and not empty, it has valid data
                rx_buffer_read_select <= 0;
                user_rx_read_valid := '1';
            elsif (rx1_locked = '0' and rx_buffer_empty(1) = '0') then
                rx_buffer_read_select <= 1;
                user_rx_read_valid := '1';
            else
                user_rx_read_valid := '0';
            end if;

            if (user_command(2) = '1' and user_rx_read_valid = '1') then --user is sending the retrieve read command
                rx_read_ptr <= rx_read_ptr + 1;
                rx_buffer_data_valid <= '1'; --signal used to switch data to the suer
                if (rx_read_ptr >= BUFFER_DEPTH - 1) then
                    rx_read_ptr <= 0;--reset read pointer
                    rx_buffer_empty(rx_buffer_read_select) <= '1'; --selected buffer is now empty
                end if;
            else
                rx_buffer_data_valid <= '0';
            end if;
        end if;
    end process;

--=================================================================================================================
--logic and processes to handle rx read RAM access
    rx_buffer_reader : process (rst_n, pause, rx_read_ptr)
      --this process converts the rx buffer read pointer to the correct address
      begin
        if (rst_n = '0') then
            rx_read_addr <= (others => '0');
        elsif (rx_read_ptr < BUFFER_DEPTH) then
            rx_read_addr <= std_logic_vector(to_unsigned(rx_read_ptr,DATA_WIDTH));
        else
            rx_read_addr <= (others => '0');
        end if;
    end process;

    switch_data_to_user : process(rst_n, rx_buf_data_out, rx_buffer_data_valid, rx_buffer_read_select)
        --this process uses the rx_buffer_data_valid signal to switch the data to the user
        begin
        if (rst_n = '0') then
            data_to_user <= (others => '0');
        elsif (rx_buffer_data_valid = '1') then
            data_to_user <= rx_buf_data_out(rx_buffer_read_select);
        else
            data_to_user <= (others => '0');
        end if;
    end process;
    --=================================================================================================================

    --============================================================================
    --this selects the fis type of the incoming link data, only valid if dword is the first of a fis
    link_fis_type <= data_from_link(7 downto 0);
    --============================================================================

    --============================================================================
    user_addr_proc : process(read_error, write_error, error_address, last_read_address,rx_buffer_read_select)
      --this processes switches the address to the user
      begin
        if (read_error = '1' or write_error = '1') then
            --if an error has occurred, update the user address with the error address
            address_to_user <= error_address;
        else
            --otherwise, update user address with the address of the selected rx buffer that will be used for the current or next retrieve read
            address_to_user <= last_read_address(rx_buffer_read_select);
        end if;
    end process;
    --============================================================================
    --update status vectors
    --Device ready status is high if the ssd is ready (com error is zero), and transport is ready
    status_to_user(DEVICE_READY_INDEX) <= '1' when (transport_ready = '1' and status_from_link(c_l_comm_err) = '0') else '0';

    update_status : process(current_state, tx_buffer_full, rx_buffer_full, rx_buffer_empty, tx_buffer_empty,
                                                                    rx0_locked, rx1_locked,tx0_locked,tx1_locked)
      begin
        if (((tx_buffer_full(0) = '0' and tx0_locked = '0') or (tx_buffer_full(1) = '0' and tx1_locked = '0'))) then
            status_to_user(WRITE_VALID_INDEX) <= '1'; --writes are valid if at least one of the tx buffers is not full and not locked
        else
            status_to_user(WRITE_VALID_INDEX) <= '0';
        end if;

        if (((rx_buffer_empty(0) = '1' and rx0_locked = '0') or (rx_buffer_empty(1) = '1' and rx1_locked = '0'))
                                                                                and current_state = transport_idle) then
            --Send reads are valid if one of the rx buffers is empty and not locked, and the state machine is in idle
            status_to_user(SEND_READ_VALID_INDEX) <= '1';
        else
            status_to_user(SEND_READ_VALID_INDEX) <= '0';
        end if;

        if ((rx_buffer_empty(0) = '0' and rx0_locked = '0') or (rx_buffer_empty(1) = '0' and rx1_locked = '0')) then
            --retrieve reads are valid if at least one of the rx buffers is not empty and not locked
            status_to_user(RETRIEVE_READ_VALID_INDEX) <= '1';
        else
            status_to_user(RETRIEVE_READ_VALID_INDEX) <= '0';
        end if;
    end process;

    status_to_user(WRITE_ERROR_INDEX) <= write_error;
    status_to_user(READ_ERROR_INDEX) <= read_error;

    link_error <= status_from_link(c_l_transmit_bad);
    link_rdy <= status_from_link (c_l_link_ready);
    pause <= status_from_link(c_l_phy_paused);

    data_from_link_valid <= status_from_link(c_l_rcv_data_valid);
    status_to_link <= "0" & rx_from_link_ready & tx_to_link_request & "00001"; --unused status fields are set to zero
end architecture;