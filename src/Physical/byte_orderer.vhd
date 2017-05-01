
library ieee;
use ieee.std_logic_1164.ALL;
use ieee.std_logic_unsigned.all;
use ieee.numeric_std.all;

use work.sata_defines.all;

entity byte_orderer is
    port(
        rxclkout         : in  std_logic;
        rst_n            : in  std_logic;
        do_byte_order    : in  std_logic; -- Indicates when word alignment has completed and locking should begin

        rx_data          : in  std_logic_vector(31 downto 0); -- received data, unordered, but aligned when do_byte_order = '1'
        rx_datak         : in  std_logic_vector(3 downto 0); -- received datak, indicating where control character is located

        is_byte_ordered  : out std_logic; -- asserted when data is byte ordered
        rx_ordered_data  : out std_logic_vector(31 downto 0); -- output ordered data
        primitive_recvd  : out std_logic -- after byte order, this signal is asserted when the ordered data contains a primitive (checks if the last byte is a primitive-type byte)
    );
end entity byte_orderer;

architecture byte_orderer_arch of byte_orderer is
    type order_buffer is array(7 downto 0) of std_logic_vector(7 downto 0);
    signal receive_byte_buffer : order_buffer; -- store received data in here
    signal rx_datak_buffer     : std_logic_vector(7 downto 0); --store received datak indicators here
    signal rx_datak_prev : std_logic_vector(3 downto 0); -- used to check if we are receiving consistent data, to make sure byte order is valid
    signal ordered_datak : std_logic_vector(3 downto 0); -- stores the location of the primitive received when doing byte ordering
begin
    process(rxclkout, rst_n)
    begin
        if(rst_n = '0') then
            is_byte_ordered <= '0';
            receive_byte_buffer(7) <= (others => '0');
            receive_byte_buffer(6) <= (others => '0');
            receive_byte_buffer(5) <= (others => '0');
            receive_byte_buffer(4) <= (others => '0');
            receive_byte_buffer(3) <= (others => '0');
            receive_byte_buffer(2) <= (others => '0');
            receive_byte_buffer(1) <= (others => '0');
            receive_byte_buffer(0) <= (others => '0');
            rx_datak_buffer <= (others => '0');
            rx_datak_prev <= DATAK_BYTE_NONE;
         elsif(rising_edge(rxclkout)) then
            receive_byte_buffer(3 downto 0) <= (rx_data(31 downto 24), rx_data(23 downto 16), rx_data(15 downto 8), rx_data(7 downto 0)); -- store new word into 3 downto 0
            receive_byte_buffer(7 downto 4) <= receive_byte_buffer(3 downto 0); -- push old word into 7 downto 4
            rx_datak_buffer(3 downto 0) <= rx_datak;-- store new word into 3 downto 0
            rx_datak_buffer(7 downto 4) <= rx_datak_buffer(3 downto 0); -- push old word into 7 downto 4
            rx_datak_prev <= rx_datak;

            if(do_byte_order = '1') then -- wait until commanding blocks says to reallign--only want to reallign during initialization
                if(rx_datak = rx_datak_prev and rx_datak /= DATAK_BYTE_NONE) then -- need to receive a primitive, and be receiving the same order
                    is_byte_ordered <= '1'; -- assert byte order succesfful (ordered_data valid!)
                    ordered_datak <= rx_datak; -- output the received datak
                else
                    is_byte_ordered <= '0';
                end if;
            else
                is_byte_ordered <= '0';
            end if;


            if(ordered_datak = DATAK_BYTE_ZERO)  then -- choose which bytes are the ordered data.
                rx_ordered_data <= receive_byte_buffer(7) & receive_byte_buffer(6) & receive_byte_buffer(5) & receive_byte_buffer(4);
                primitive_recvd <= rx_datak_buffer(4);
            elsif(ordered_datak = DATAK_BYTE_ONE)   then
                rx_ordered_data <= receive_byte_buffer(0) & receive_byte_buffer(7) & receive_byte_buffer(6) & receive_byte_buffer(5);
                primitive_recvd <= rx_datak_buffer(5);
            elsif(ordered_datak = DATAK_BYTE_TWO)   then
                rx_ordered_data <= receive_byte_buffer(1) & receive_byte_buffer(0) & receive_byte_buffer(7) & receive_byte_buffer(6);
                primitive_recvd <= rx_datak_buffer(6);
            elsif(ordered_datak = DATAK_BYTE_THREE) then
                rx_ordered_data <= receive_byte_buffer(2) & receive_byte_buffer(1) & receive_byte_buffer(0) & receive_byte_buffer(7);
                primitive_recvd <= rx_datak_buffer(7);
            else
                rx_ordered_data <= (others => '0');
                primitive_recvd <= '0';
            end if;
        end if;
    end process;
end architecture byte_orderer_arch;
