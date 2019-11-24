# Makefile For VCS

all:
	@echo "VCS Verification ENV"

cmp:
	@vcs -LDFLAGS -Wl,--no-as-needed -debug_pp +lint=TFIPC-L +lint=PCWM -f ../rtl/filelist.f -top ZYNQ_PRV32_TB -l vcs.log

run:
	@./simv -l simv.log

dve:
	@dve -vpd ZYNQ_PRV32_TB.vpd &

clean:
	@rm -rf ucli.key csrc simv simv.daidir vcs.log simv.log ZYNQ_PRV32_TB.vpd DVEfiles

