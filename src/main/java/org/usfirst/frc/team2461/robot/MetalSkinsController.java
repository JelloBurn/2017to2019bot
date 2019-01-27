package org.usfirst.frc.team2461.robot;

import edu.wpi.first.wpilibj.XboxController;

public class MetalSkinsController extends XboxController
{
	private double precision = 0.15;
	
	public MetalSkinsController(final int port)
	{
		super(port);
	}
	
	@Override
	public double getX(Hand hand)
	{
		if(super.getX(hand) < precision && super.getX(hand) > -precision)
		{
			return 0;
		}
		else
		{
			return super.getX(hand);
		}
	}
	
	@Override
	public double getY(Hand hand)
	{
		if(super.getY(hand) < precision && super.getY(hand) > -precision)
		{
			return 0;
		}
		else
		{
			return super.getY(hand);
		}
	}
}
