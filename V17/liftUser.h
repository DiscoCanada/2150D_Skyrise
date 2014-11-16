task armComp()
{
	if(vexRT[Btn5U] == 1) {
			armPo(127);
		}
		else if(vexRT[Btn5D] == 1) {
			armPo(-127);
		}
		else {
			armPo(0);
		}
}
