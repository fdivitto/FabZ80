-- Copyright 2018 Fabrizio Di Vittorio (fdivitto2013@gmail.com)

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;


entity GAL is
	port (Z80_RD:       in std_logic;
			  Z80_WR:       in std_logic;
				Z80_MREQ:     in std_logic;
				Z80_IORQ:     in std_logic;
				CODE_INJECT:  in std_logic;   -- low if code injecting
				ATMEGA_READY: in std_logic;   -- low if atmega is ready to send or recv data
				RAM_SELECT:   out std_logic;  -- low if RAM chip selected
				Z80_WAIT:     out std_logic);

	attribute LOC : string;
	attribute LOC of Z80_RD:       signal is "P1";
	attribute LOC of Z80_WR:       signal is "P2";
	attribute LOC of Z80_MREQ:     signal is "P3";
	attribute LOC of Z80_IORQ:     signal is "P4";
	attribute LOC of CODE_INJECT:  signal is "P5";
	attribute LOC of ATMEGA_READY: signal is "P6";
	attribute LOC of RAM_SELECT:   signal is "P18";
	attribute LOC of Z80_WAIT:     signal is "P19";
end GAL;


architecture Behavioral of GAL is
begin

	process (Z80_MREQ, Z80_RD, CODE_INJECT)
	begin
		if (Z80_MREQ = '0' and Z80_RD = '0' and CODE_INJECT = '1') or   -- RAM selected when reading from memory and not in code injecting
		   (Z80_MREQ = '0' and Z80_WR = '0') then                       -- RAM selected when writing to memory
			RAM_SELECT <= '0';
		else
			RAM_SELECT <= '1';
		end if;
	end process;

	process (Z80_IORQ, Z80_MREQ, Z80_RD, Z80_WR, CODE_INJECT, ATMEGA_READY)
	begin
		if ATMEGA_READY = '1' and																			    -- atmega is Not ready
		   ((Z80_IORQ = '0' and (Z80_RD = '0' or Z80_WR = '0')) or        -- I/O request (handled by atmega)
		   (Z80_MREQ = '0' and Z80_RD = '0' and CODE_INJECT = '0')) then  -- read RAM while code injecting (handled by atmega)
		  Z80_WAIT <= '0';
		else
			Z80_WAIT <= '1';
		end if;
	end process;

end Behavioral;
