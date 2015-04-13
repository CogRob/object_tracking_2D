all: help

help:
	@echo ""
	@echo "-- Help Menu"
	@echo ""
	@echo "   1. make build            - build all cogrob images"
	@echo "   1. make pull             - pull all cogrob images"
	@echo "   1. make clean            - remove all cogrob images"
	@echo "   2. make terminator       - run terminator on ebt-dox"
	@echo "   2. make bash             - run bash on ebt-dox"
	@echo ""

build:
	@docker build --tag=cogrob/ebt-dep dep/.
	@docker build --tag=cogrob/ebt-dev dev/.
	@docker build --tag=cogrob/ebt-gui gui/.
	@docker build --tag=cogrob/ebt-dox dox/.
	@docker build --tag=cogrob/ebt-demo demo/.
	# @( cd nvidia ; bash build.sh)
	# @docker build --tag=cogrob/ebt-nvidia nvidia/.

pull:
	@docker pull cogrob/ebt-dep
	@docker pull cogrob/ebt-dev
	@docker pull cogrob/ebt-gui
	@docker pull cogrob/ebt-dox
	@docker pull cogrob/ebt-demo
	# @docker pull cogrob/ebt-nvidia
	
clean:
	@docker rmi -f cogrob/ebt-dep
	@docker rmi -f cogrob/ebt-dev
	@docker rmi -f cogrob/ebt-gui
	@docker rmi -f cogrob/ebt-dox
	@docker rmi -f cogrob/ebt-demo
	# @docker rmi -f cogrob/ebt-nvidia

terminator bash:
	@bash run.sh -i -t cogrob/ebt-dox $@