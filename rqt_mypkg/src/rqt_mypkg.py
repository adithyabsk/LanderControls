import sys;

from rqt_gui.main import Main;

main = Main();
sys.exit(main.main(sys.argv, standalone="rqt_mypkg"));
