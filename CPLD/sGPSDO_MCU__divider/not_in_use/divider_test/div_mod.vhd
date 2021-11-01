----------------------------------------------------------------------------------
-- Company: 
-- Engineer: 
-- 
-- Create Date:    21:52:39 10/31/2021 
-- Design Name: 
-- Module Name:    div_mod - Behavioral 
-- Project Name: 
-- Target Devices: 
-- Tool versions: 
-- Description: 
--
-- Dependencies: 
--
-- Revision: 
-- Revision 0.01 - File Created
-- Additional Comments: 
--
----------------------------------------------------------------------------------
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;

-- Uncomment the following library declaration if using
-- arithmetic functions with Signed or Unsigned values
use IEEE.NUMERIC_STD.ALL;

-- Uncomment the following library declaration if instantiating
-- any Xilinx primitives in this code.
library UNISIM;
use UNISIM.VComponents.all;

entity div_mod is
    Port ( phase_in 	: in	STD_LOGIC;
           reset    	: in	STD_LOGIC;
           div_out 	: out	STD_LOGIC);
end div_mod;

architecture Behavioral of div_mod is
   component BUFG is
   port (
     I					: in	STD_LOGIC;
     O					: out	STD_LOGIC);
	end component BUFG;

	signal clk 			: STD_LOGIC;
	signal cnt 			: STD_LOGIC_VECTOR( 4 downto 0 );

begin
	clk_g: component BUFG
	port map (
		I					=> phase_in,
		O					=> clk);


	process(clk, reset)
	begin
		if (reset = '1') then
			cnt <= (others => '0');
		
		elsif (clk'event and clk = '1') then
	
			if (cnt = "11110") then
				cnt <= (others => '0');
			else
				cnt <= std_logic_vector(to_unsigned((to_Integer(unsigned(cnt)) + 1), cnt'length));
			end if;
		
			if (to_Integer(unsigned(cnt)) < 16) then
				div_out <= '1';
			else
				div_out <= '0';
			end if;
		end if;
	end process;

end Behavioral;

