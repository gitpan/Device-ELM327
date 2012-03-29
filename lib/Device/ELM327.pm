package Device::ELM327;

use strict;
use warnings;

use Data::Dumper;

my $null = "\x0";
my $lf = "\xa";
my $cr = "\xd";

my $on = 1;
my $off = 0;

my $max_ports_to_search = 32;

my @has_sub_command = (0, 1, 1, 0, 0, 1, 1, 0, 1, 1, 0);

BEGIN
{
  if ($^O eq "MSWin32")
  {
    require Win32::SerialPort;
  }
  else
  {
    require Device::SerialPort;
    Device::SerialPort->import qw( :PARAM :STAT 0.07 );
  }
}

=head1 NAME

Device::ELM327 - Methods for reading OBD data with an ELM327 module.

=head1 VERSION

Version 0.03

=cut

our $VERSION = '0.03';

#*****************************************************************

=head1 SYNOPSIS

This module provides a Perl interface to a device containing an Elm Electronics ELM327 OBD Interpreter and provides access to the following functions:

	Read OBD parameters and extract individual values from results.
	Read OBD Trouble Codes and expand them to their full form.
	Reset OBD Trouble Codes.
	Read ELM327 parameters.
	Write and write the ELM327 data byte.
	Calibrate ELM327 Voltage.
	Switchable diagnostic trace and replay function for debugging.

The module is written entirely in Perl and works with both Linux and Windows. Depending on which operating system it is run on it uses either the Win32::SerialPort or Device::SerialPort module so it should work on any platform that supports one of them.

  use Device::ELM327;

  my $obd = Device::ELM327->new();

  # Read status information...
  $obd->Show("ELM identity");
  $obd->Show("Vehicle Identification Number");
  $obd->Show("Engine RPM");
  $obd->ShowTroubleCodes();

  undef $obd;


=head1 SUBROUTINES/METHODS


=head2 new - the constructor for the module.

To open the device and have it search for an ELM module:

	my $obd = Device::ELM327->new();

If you know the port name (e.g. 'COM5', '/dev/ttyUSB7' etc) it may be 
quicker to pass it into the function:

	my $obd = Device::ELM327->new($port_name);

If you want extra debugging information, it can be enabled by setting 
$debug_level to a positive number in the range 1 to 3, with more 
information being displayed as the number increases:

	my $obd = Device::ELM327->new($port_name, $debug_level);

A value of either undef or "" can be passed for the $port_name:

	my $obd = Device::ELM327->new("", $debug_level);

The module can replay previously captured debugging information:

	my $obd = Device_ELM327->new(undef, $debug_level, $replay_filename);

To produce a file containing replayable data simply set $debug_level to
1 or higher and pipe the output to a text file:

	perl test.pl>test_output.txt

=cut

sub new
{
	my ($class, $port_name, $debug_level, $replay_filename) = @_;
	my $self = bless { }, $class;

	$self->{'version'} = $VERSION;

	$self->{'debug_level'} = 0;
	if (defined($debug_level))
	{
	  $self->{'debug_level'} = $debug_level;
	}

  $self->{'ELM_type'} = "NONE";
	$self->{'replay_file'} = 0;
	$self->{'replay_response'} = ();
 
	$self->{'last_command'} = 0;
	$self->{'last_sub_command'} = 0;
	$self->{'response'} = ();
	$self->{'response_length'} = 0;
	$self->{'results'} = {};
	
	$self->{'number_of_results'} = 0;
	$self->{'command_addresses'} = [];
	$self->{'command_results'} = [];

	$self->{'trouble_codes'} = [];

  $self->{'get'} = {
            "ELM identity" => { command => "AT I", available => 1, result => [{type => "AT", modifier =>"", unit=>""}] },
            "Stored data byte" => { command => "AT RD", available => 1, result => [{type => "AT", modifier =>"", unit=>""}] },
            "Input Voltage" => { command => "AT RV", available => 1, result => [{type => "AT", modifier =>'=~ s/V//', unit=>"V"}] },
            "Ignition state" => { command => "AT IGN", available => 1, result => [{type => "AT", modifier =>"", unit=>""}] },

            "01 PIDs supported (01-20)" => { command => "01 00", available => 1,
            result => [
            {name => "Monitor status since DTCs cleared", type => "bool_0", modifier => "&128", unit => ""},
            {name => "DTC that caused required freeze frame data storage", type => "bool_0", modifier => "&64", unit => ""},
            {name => "Fuel systems status", type => "bool_0", modifier => "&32", unit => ""},
            {name => "Calculated LOAD Value!", type => "bool_0", modifier => "&16", unit => ""},
            {name => "Engine Coolant Temperature", type => "bool_0", modifier => "&8", unit => ""},
            {name => "Short Term Fuel Trim - Bank 1/3", type => "bool_0", modifier => "&4", unit => ""},
            {name => "Long Term Fuel Trim - Bank 1/3", type => "bool_0", modifier => "&2", unit => ""},
            {name => "Short Term Fuel Trim - Bank 2/4", type => "bool_0", modifier => "&1", unit => ""},

            {name => "Long Term Fuel Trim - Bank 2/4", type => "bool_1", modifier => "&128", unit => ""},
            {name => "Fuel Rail Pressure (gauge)", type => "bool_1", modifier => "&64", unit => ""},
            {name => "Intake Manifold Absolute Pressure", type => "bool_1", modifier => "&32", unit => ""},
            {name => "Engine RPM", type => "bool_1", modifier => "&16", unit => ""},
            {name => "Vehicle Speed Sensor", type => "bool_1", modifier => "&8", unit => ""},
            {name => "Ignition Timing Advance for #1 Cylinder", type => "bool_1", modifier => "&4", unit => ""},
            {name => "Intake Air Temperature", type => "bool_1", modifier => "&2", unit => ""},
            {name => "Air Flow Rate from Mass Air Flow Sensor", type => "bool_1", modifier => "&1", unit => ""},
            
            {name => "Absolute Throttle Position", type => "bool_2", modifier => "&128", unit => ""},
            {name => "Commanded Secondary Air Status", type => "bool_2", modifier => "&64", unit => ""},
            {name => "Location of oxygen sensors 13", type => "bool_2", modifier => "&32", unit => ""},
            {name => "Bank 1 - Sensor 1", type => "bool_2", modifier => "&16", unit => ""},
            {name => "Bank 1 - Sensor 2", type => "bool_2", modifier => "&8", unit => ""},
            
            {name => "Bank 1 - Sensor 3", type => "bool_2", modifier => "&4", unit => ""},
            {name => "Bank 1 - Sensor 4", type => "bool_2", modifier => "&2", unit => ""},
            {name => "Bank 2 - Sensor 1 13", type => "bool_2", modifier => "&1", unit => ""},
            
            {name => "Bank 2 - Sensor 2 13", type => "bool_3", modifier => "&128", unit => ""},
            {name => "Bank 2 - Sensor 3", type => "bool_3", modifier => "&64", unit => ""},
            {name => "Bank 2 - Sensor 4", type => "bool_3", modifier => "&32", unit => ""},
            {name => "Bank 3 - Sensor 2", type => "bool_3", modifier => "&128", unit => ""},
            {name => "Bank 4 - Sensor 1", type => "bool_3", modifier => "&64", unit => ""},
            {name => "Bank 4 - Sensor 2", type => "bool_3", modifier => "&32", unit => ""},
            {name => "OBD requirements to which vehicle is designed", type => "bool_3", modifier => "&16", unit => ""},
            {name => "Location of oxygen sensors 1D", type => "bool_3", modifier => "&8", unit => ""},
            {name => "Bank 2 - Sensor 1 1D", type => "bool_2", modifier => "&4", unit => ""},
            {name => "Bank 2 - Sensor 2 1D", type => "bool_2", modifier => "&2", unit => ""},
            {name => "Bank 3 - Sensor 1", type => "bool_2", modifier => "&1", unit => ""},
            {name => "Auxiliary Input Status", type => "bool_3", modifier => "&4", unit => ""},
            {name => "Time Since Engine Start", type => "bool_3", modifier => "&2", unit => ""},
            {name => "01 PIDs supported (21-40)", type => "bool_3", modifier => "&1", unit => ""},
            ] },
            
            "Monitor status since DTCs cleared" => { command => "01 01",
            result => [
            {name => "Number of DTCs stored in this ECU", type => "byte_0", modifier => "&127", unit => ""},
            {name => "Malfunction Indicator Lamp (MIL) Status", type => "bool_0", modifier => "&128", unit => ""},
            {name => "Misfire monitoring supported", type => "bool_1", modifier => "&1", unit => ""},
            {name => "Fuel system monitoring supported", type => "bool_1", modifier => "&2", unit => ""},
            {name => "Comprehensive component monitoring supported", type => "bool_1", modifier => "&4", unit => ""},
            {name => "Misfire monitoring complete", type => "bool_1", modifier => "&16", unit => ""},
            {name => "Fuel system monitoring complete", type => "bool_1", modifier => "&32", unit => ""},
            {name => "Comprehensive component monitoring complete", type => "bool_1", modifier => "&64", unit => ""},
            {name => "Catalyst monitoring supported", type => "bool_2", modifier => "&1", unit => ""},
            {name => "Heated catalyst monitoring supported", type => "bool_2", modifier => "&2", unit => ""},
            {name => "Evaporative system monitoring supported", type => "bool_2", modifier => "&4", unit => ""},
            {name => "Secondary air system monitoring supported", type => "bool_2", modifier => "&8", unit => ""},
            {name => "A/C system refrigerant monitoring supported", type => "bool_2", modifier => "&16", unit => ""},
            {name => "Oxygen sensor monitoring supported", type => "bool_2", modifier => "&32", unit => ""},
            {name => "Oxygen sensor heater monitoring supported", type => "bool_2", modifier => "&64", unit => ""},
            {name => "EGR system monitoring supported", type => "bool_2", modifier => "&128", unit => ""},
            {name => "Catalyst monitoring complete", type => "bool_3", modifier => "&1", unit => ""},
            {name => "Heated catalyst monitoring complete", type => "bool_3", modifier => "&2", unit => ""},
            {name => "Evaporative system monitoring complete", type => "bool_3", modifier => "&4", unit => ""},
            {name => "Secondary air system monitoring complete", type => "bool_3", modifier => "&8", unit => ""},
            {name => "A/C system refrigerant monitoring complete", type => "bool_3", modifier => "&16", unit => ""},
            {name => "Oxygen sensor monitoring complete", type => "bool_3", modifier => "&32", unit => ""},
            {name => "Oxygen sensor heater monitoring complete", type => "bool_3", modifier => "&64", unit => ""},
            {name => "EGR system monitoring complete", type => "bool_3", modifier => "&128", unit => ""},
            ] },
            "DTC that caused required freeze frame data storage" => { command => "01 02", result => [{type => "word_0", modifier => "+0", unit => ""}] },
            "Fuel systems status" => { command => "01 03",
            result => [
            {name => "Fuel system 1 status", type => "byte_0", modifier => "&31", unit => "",
            alternatives => [
            {value => 1, meaning => "Open loop - has not yet satisfied conditions to go closed loop"},
            {value => 2, meaning => "Closed loop - using oxygen sensor(s) as feedback for fuel control"},
            {value => 4, meaning => "Open loop due to driving conditions (e.g., power enrichment, deceleration enleanment)"},
            {value => 8, meaning => "Open loop - due to detected system fault"},
            {value => 16, meaning => "Closed loop, but fault with at least one oxygen sensor - may be using single oxygen sensor for fuel control"}
            ]
            },
            {name => "Fuel system 2 status", type => "byte_1", modifier => "&31", unit => "",
            alternatives => [
            {value => 1, meaning => "Open loop - has not yet satisfied conditions to go closed loop"},
            {value => 2, meaning => "Closed loop - using oxygen sensor(s) as feedback for fuel control"},
            {value => 4, meaning => "Open loop due to driving conditions (e.g., power enrichment, deceleration enleanment)"},
            {value => 8, meaning => "Open loop - due to detected system fault"},
            {value => 16, meaning => "Closed loop, but fault with at least one oxygen sensor - may be using single oxygen sensor for fuel control"}
            ]
            }
            ] },

            "Calculated LOAD Value!" => { command => "01 04", result => [{type => "byte_0", modifier => "*100/255", unit => "%"}] },
            "Engine Coolant Temperature" => { command => "01 05", result => [{type => "byte_0", modifier => "-40", unit => "°C"}] },

            "Short Term Fuel Trim - Bank 1/3" => { command => "01 06",
            result => [
            {name => "Short Term Fuel Trim - Bank 1", type => "signed_byte_0", modifier => "*100/128", unit => "%"},
            {name => "Short Term Fuel Trim - Bank 3", type => "signed_byte_1", modifier => "*100/128", unit => "%"},
            ] },

            "Long Term Fuel Trim - Bank 1/3" => { command => "01 07",
            result => [
            {name => "Long Term Fuel Trim - Bank 1", type => "signed_byte_0", modifier => "*100/128", unit => "%"},
            {name => "Long Term Fuel Trim - Bank 3", type => "signed_byte_1", modifier => "*100/128", unit => "%"},
            ] },

            "Short Term Fuel Trim - Bank 2/4" => { command => "01 08",
            result => [
            {name => "Short Term Fuel Trim - Bank 2", type => "signed_byte_0", modifier => "*100/128", unit => "%"},
            {name => "Short Term Fuel Trim - Bank 4", type => "signed_byte_1", modifier => "*100/128", unit => "%"},
            ] },

            "Long Term Fuel Trim - Bank 2/4" => { command => "01 09",
            result => [
            {name => "Long Term Fuel Trim - Bank 2", type => "signed_byte_0", modifier => "*100/128", unit => "%"},
            {name => "Long Term Fuel Trim - Bank 4", type => "signed_byte_1", modifier => "*100/128", unit => "%"},
            ] },

            
            "Fuel Rail Pressure (gauge)" => { command => "01 0A", result => [{type => "byte_0", modifier => "*3", unit => "kPa"}] },
            "Intake Manifold Absolute Pressure" => { command => "01 0B", result => [{type => "byte_0", modifier => "*1", unit => "kPa"}] },
            "Engine RPM" => { command => "01 0C", result => [{type => "word_0", modifier => "/4", unit => "RPM"}] },
            "Vehicle Speed Sensor" => { command => "01 0D", result => [{type => "byte_0", modifier => "+0", unit => "km/h"}] },
            "Ignition Timing Advance for #1 Cylinder" => { command => "01 0E", result => [{type => "signed_byte_0", modifier => "/2", unit => "°"}] },
            "Intake Air Temperature" => { command => "01 0F", result => [{type => "byte_0", modifier => "-40", unit => "°C"}] },
            "Air Flow Rate from Mass Air Flow Sensor" => { command => "01 10", result => [{type => "word_0", modifier => "/100", unit => "g/s"}] },
            "Absolute Throttle Position" => { command => "01 11", result => [{type => "byte_0", modifier => "*100/255", unit => "%"}] },
            "Commanded Secondary Air Status" => { command => "01 12", result => [{type => "byte_0", modifier => "*100/255", unit => "", 
            alternatives => [
            {value => 1, meaning => "upstream of first catalytic converter"},
            {value => 2, meaning => "downstream of first catalytic converter inlet"},
            {value => 4, meaning => "atmosphere / off"}
            ] }
            ] },

            "Location of oxygen sensors 13" => { command => "01 13",
            result => [
            {name => "Bank 1 - Sensor 1 present", type => "bool_0", modifier => "&1", unit => ""},
            {name => "Bank 1 - Sensor 2 present", type => "bool_0", modifier => "&2", unit => ""},
            {name => "Bank 1 - Sensor 3 present", type => "bool_0", modifier => "&4", unit => ""},
            {name => "Bank 1 - Sensor 4 present", type => "bool_0", modifier => "&8", unit => ""},
            {name => "Bank 2 - Sensor 1 present", type => "bool_0", modifier => "&16", unit => ""},
            {name => "Bank 2 - Sensor 2 present", type => "bool_0", modifier => "&32", unit => ""},
            {name => "Bank 2 - Sensor 3 present", type => "bool_0", modifier => "&64", unit => ""},
            {name => "Bank 2 - Sensor 4 present", type => "bool_0", modifier => "&128", unit => ""}
            ] },

            "Bank 1 - Sensor 1" => { command => "01 14",
            result => [
            {name => "Oxygen Sensor Output Voltage (Bx-Sy)", type => "byte_0", modifier => "*0.005", unit => "V"},
            {name => "Short Term Fuel Trim (Bx-Sy)", type => "byte_1", modifier => "*100/255", unit => "%"}
            ] },
            
            "Bank 1 - Sensor 2" => { command => "01 15",
            result => [
            {name => "Oxygen Sensor Output Voltage (Bx-Sy)", type => "byte_0", modifier => "*0.005", unit => "V"},
            {name => "Short Term Fuel Trim (Bx-Sy)", type => "byte_1", modifier => "*100/255", unit => "%"}
            ] },

            "Bank 1 - Sensor 3" => { command => "01 16",
            result => [
            {name => "Oxygen Sensor Output Voltage (Bx-Sy)", type => "byte_0", modifier => "*0.005", unit => "V"},
            {name => "Short Term Fuel Trim (Bx-Sy)", type => "byte_1", modifier => "*100/255", unit => "%"}
            ] },

            "Bank 1 - Sensor 4" => { command => "01 17",
            result => [
            {name => "Oxygen Sensor Output Voltage (Bx-Sy)", type => "byte_0", modifier => "*0.005", unit => "V"},
            {name => "Short Term Fuel Trim (Bx-Sy)", type => "byte_1", modifier => "*100/255", unit => "%"}
            ] },

            "Bank 2 - Sensor 1 13" => { command => "01 18",
            result => [
            {name => "Oxygen Sensor Output Voltage (Bx-Sy)", type => "byte_0", modifier => "*0.005", unit => "V"},
            {name => "Short Term Fuel Trim (Bx-Sy)", type => "byte_1", modifier => "*100/255", unit => "%"}
            ] },

            "Bank 2 - Sensor 2 13" => { command => "01 19",
            result => [
            {name => "Oxygen Sensor Output Voltage (Bx-Sy)", type => "byte_0", modifier => "*0.005", unit => "V"},
            {name => "Short Term Fuel Trim (Bx-Sy)", type => "byte_1", modifier => "*100/255", unit => "%"}
            ] },

            "Bank 2 - Sensor 3" => { command => "01 1A",
            result => [
            {name => "Oxygen Sensor Output Voltage (Bx-Sy)", type => "byte_0", modifier => "*0.005", unit => "V"},
            {name => "Short Term Fuel Trim (Bx-Sy)", type => "byte_1", modifier => "*100/255", unit => "%"}
            ] },

            "Bank 2 - Sensor 4" => { command => "01 1B",
            result => [
            {name => "Oxygen Sensor Output Voltage (Bx-Sy)", type => "byte_0", modifier => "*0.005", unit => "V"},
            {name => "Short Term Fuel Trim (Bx-Sy)", type => "byte_1", modifier => "*100/255", unit => "%"}
            ] },


            "Bank 2 - Sensor 1 1D" => { command => "01 16",
            result => [
            {name => "Oxygen Sensor Output Voltage (Bx-Sy)", type => "byte_0", modifier => "*0.005", unit => "V"},
            {name => "Short Term Fuel Trim (Bx-Sy)", type => "byte_1", modifier => "*100/255", unit => "%"}
            ] },

            "Bank 2 - Sensor 2 1D" => { command => "01 17",
            result => [
            {name => "Oxygen Sensor Output Voltage (Bx-Sy)", type => "byte_0", modifier => "*0.005", unit => "V"},
            {name => "Short Term Fuel Trim (Bx-Sy)", type => "byte_1", modifier => "*100/255", unit => "%"}
            ] },
            
            "Bank 3 - Sensor 1" => { command => "01 18",
            result => [
            {name => "Oxygen Sensor Output Voltage (Bx-Sy)", type => "byte_0", modifier => "*0.005", unit => "V"},
            {name => "Short Term Fuel Trim (Bx-Sy)", type => "byte_1", modifier => "*100/255", unit => "%"}
            ] },

            "Bank 3 - Sensor 2" => { command => "01 19",
            result => [
            {name => "Oxygen Sensor Output Voltage (Bx-Sy)", type => "byte_0", modifier => "*0.005", unit => "V"},
            {name => "Short Term Fuel Trim (Bx-Sy)", type => "byte_1", modifier => "*100/255", unit => "%"}
            ] },

            "Bank 4 - Sensor 1" => { command => "01 1A",
            result => [
            {name => "Oxygen Sensor Output Voltage (Bx-Sy)", type => "byte_0", modifier => "*0.005", unit => "V"},
            {name => "Short Term Fuel Trim (Bx-Sy)", type => "byte_1", modifier => "*100/255", unit => "%"}
            ] },

            "Bank 4 - Sensor 2" => { command => "01 1B",
            result => [
            {name => "Oxygen Sensor Output Voltage (Bx-Sy)", type => "byte_0", modifier => "*0.005", unit => "V"},
            {name => "Short Term Fuel Trim (Bx-Sy)", type => "byte_1", modifier => "*100/255", unit => "%"}
            ] },


            "OBD requirements to which vehicle is designed" => { command => "01 1C", result => [{type => "byte_0", modifier => "+0", unit => "",
            alternatives => [
            {value => 1, meaning => "OBD II (California ARB)"},
            {value => 2, meaning => "OBD (Federal EPA)"},
            {value => 3, meaning => "OBD and OBD II"},
            {value => 4, meaning => "OBD I"},
            {value => 5, meaning => "Not OBD compliant"},
            {value => 6, meaning => "EOBD"},
            {value => 7, meaning => "EOBD and OBD II"},
            {value => 8, meaning => "EOBD and OBD"},
            {value => 9, meaning => "EOBD, OBD and OBD II"},
            {value => 10, meaning => "JOBD"},
            {value => 11, meaning => "JOBD and OBD II"},
            {value => 12, meaning => "JOBD and EOBD"},
            {value => 13, meaning => "JOBD, EOBD, and OBD II"}
            ] }
            ] },
            
            "Location of oxygen sensors 1D" => { command => "01 1D",
            result => [
            {name => "Bank 1 - Sensor 1 present", type => "bool_0", modifier => "&1", unit => ""},
            {name => "Bank 1 - Sensor 2 present", type => "bool_0", modifier => "&2", unit => ""},
            {name => "Bank 2 - Sensor 1 present", type => "bool_0", modifier => "&4", unit => ""},
            {name => "Bank 2 - Sensor 2 present", type => "bool_0", modifier => "&8", unit => ""},
            {name => "Bank 3 - Sensor 1 present", type => "bool_0", modifier => "&16", unit => ""},
            {name => "Bank 3 - Sensor 2 present", type => "bool_0", modifier => "&32", unit => ""},
            {name => "Bank 4 - Sensor 1 present", type => "bool_0", modifier => "&64", unit => ""},
            {name => "Bank 4 - Sensor 2 present", type => "bool_0", modifier => "&128", unit => ""},
            ] },
            
            "Auxiliary Input Status" => { command => "01 1E", result => [{name => "Power Take Off (PTO) Status", type => "bool_0", modifier => "&1", unit => ""}] },
            "Time Since Engine Start" => { command => "01 1F", result => [{type => "word_0", modifier => "*10", unit => "s"}] },

            "01 PIDs supported (21-40)" => { command => "01 20",
            result => [
            {name => "Distance Travelled While MIL is Activated", type => "bool_0", modifier => "&128", unit => ""},
            {name => "Fuel Rail Pressure relative to manifold vacuum", type => "bool_0", modifier => "&64", unit => ""},
            {name => "Fuel Rail Pressure", type => "bool_0", modifier => "&32", unit => ""},
            {name => "Bank 1 - Sensor 1 (wide range O2S)", type => "bool_0", modifier => "&16", unit => ""},
            {name => "Bank 1 - Sensor 2 (wide range O2S)", type => "bool_0", modifier => "&8", unit => ""},
            
            {name => "Bank 1 - Sensor 3 (wide range O2S)", type => "bool_0", modifier => "&4", unit => ""},
            {name => "Bank 1 - Sensor 4 (wide range O2S)", type => "bool_0", modifier => "&2", unit => ""},
            {name => "Bank 2 - Sensor 1 (wide range O2S) 13", type => "bool_0", modifier => "&1", unit => ""},
            {name => "Bank 2 - Sensor 1 (wide range O2S) 1D", type => "bool_0", modifier => "&4", unit => ""},
            {name => "Bank 2 - Sensor 2 (wide range O2S) 1D", type => "bool_0", modifier => "&2", unit => ""},
            {name => "Bank 3 - Sensor 1 (wide range O2S)", type => "bool_0", modifier => "&1", unit => ""},

            {name => "Bank 2 - Sensor 2 (wide range O2S) 13", type => "bool_1", modifier => "&128", unit => ""},
            {name => "Bank 2 - Sensor 3 (wide range O2S)", type => "bool_1", modifier => "&64", unit => ""},
            {name => "Bank 2 - Sensor 4 (wide range O2S)", type => "bool_1", modifier => "&32", unit => ""},
            {name => "Bank 3 - Sensor 2 (wide range O2S)", type => "bool_1", modifier => "&128", unit => ""},
            {name => "Bank 4 - Sensor 1 (wide range O2S)", type => "bool_1", modifier => "&64", unit => ""},
            {name => "Bank 4 - Sensor 2 (wide range O2S)", type => "bool_1", modifier => "&32", unit => ""},
            
            {name => "Commanded EGR", type => "bool_1", modifier => "&16", unit => ""},
            {name => "EGR Error", type => "bool_1", modifier => "&8", unit => ""},
            {name => "Commanded Evaporative Purge", type => "bool_1", modifier => "&4", unit => ""},
            {name => "Fuel Level Input", type => "bool_1", modifier => "&2", unit => ""},
            {name => "Number of warm-ups since diagnostic trouble codes cleared", type => "bool_1", modifier => "&1", unit => ""},
            
            {name => "Distance since diagnostic trouble codes cleared", type => "bool_2", modifier => "&128", unit => ""},
            {name => "Evap System Vapor Pressure", type => "bool_2", modifier => "&64", unit => ""},
            {name => "Barometric Pressure", type => "bool_2", modifier => "&32", unit => ""},
            {name => "Bank 1 - Sensor 1 (wide range O2S) current", type => "bool_2", modifier => "&16", unit => ""},
            {name => "Bank 1 - Sensor 2 (wide range O2S) current", type => "bool_2", modifier => "&8", unit => ""},
            {name => "Bank 1 - Sensor 3 (wide range O2S) current", type => "bool_2", modifier => "&4", unit => ""},
            {name => "Bank 1 - Sensor 4 (wide range O2S) current", type => "bool_2", modifier => "&2", unit => ""},
            {name => "Bank 2 - Sensor 1 (wide range O2S) current 13", type => "bool_2", modifier => "&1", unit => ""},

            {name => "Bank 2 - Sensor 1 (wide range O2S) current 1D", type => "bool_2", modifier => "&4", unit => ""},
            {name => "Bank 2 - Sensor 2 (wide range O2S) current 1D", type => "bool_2", modifier => "&2", unit => ""},
            {name => "Bank 3 - Sensor 1 (wide range O2S) current", type => "bool_2", modifier => "&1", unit => ""},
            
            {name => "Bank 2 - Sensor 2 (wide range O2S) current 13", type => "bool_3", modifier => "&128", unit => ""},
            {name => "Bank 2 - Sensor 3 (wide range O2S) current", type => "bool_3", modifier => "&64", unit => ""},
            {name => "Bank 2 - Sensor 4 (wide range O2S) current", type => "bool_3", modifier => "&32", unit => ""},
            {name => "Bank 3 - Sensor 2 (wide range O2S) current", type => "bool_3", modifier => "&128", unit => ""},
            {name => "Bank 4 - Sensor 1 (wide range O2S) current", type => "bool_3", modifier => "&64", unit => ""},
            {name => "Bank 4 - Sensor 2 (wide range O2S) current", type => "bool_3", modifier => "&32", unit => ""},
            {name => "Catalyst Temperature Bank 1, Sensor 1", type => "bool_3", modifier => "&16", unit => ""},
            {name => "Catalyst Temperature Bank 1, Sensor 2", type => "bool_3", modifier => "&8", unit => ""},
            {name => "Catalyst Temperature Bank 2, Sensor 1", type => "bool_3", modifier => "&4", unit => ""},
            {name => "Catalyst Temperature Bank 2, Sensor 2", type => "bool_3", modifier => "&2", unit => ""},
            {name => "01 PIDs supported (41-60)", type => "bool_3", modifier => "&1", unit => ""},
            ] },
            
            "Distance Travelled While MIL is Activated" => { command => "01 21", result => [{type => "word_0", modifier => "+0", unit => "km"}] },
            "Fuel Rail Pressure relative to manifold vacuum" => { command => "01 22", result => [{type => "word_0", modifier => "*0.079", unit => "kPa"}] },
            "Fuel Rail Pressure" => { command => "01 23", result => [{type => "word_0", modifier => "*10", unit => "kPa"}] },

            "Bank 1 - Sensor 1 (wide range O2S)" => { command => "01 24",
            result => [
            {name => "Equivalence Ratio (lambda) (Bx-Sy)", type => "word_0", modifier => "*0.0000305", unit => ""},
            {name => "Oxygen Sensor Voltage (Bx-Sy)", type => "word_1", modifier => "*0.000122", unit => "V"}
            ] },
            
            "Bank 1 - Sensor 2 (wide range O2S)" => { command => "01 25",
            result => [
            {name => "Equivalence Ratio (lambda) (Bx-Sy)", type => "word_0", modifier => "*0.0000305", unit => ""},
            {name => "Oxygen Sensor Voltage (Bx-Sy)", type => "word_1", modifier => "*0.000122", unit => "V"}
            ] },

            "Bank 1 - Sensor 3 (wide range O2S)" => { command => "01 26",
            result => [
            {name => "Equivalence Ratio (lambda) (Bx-Sy)", type => "word_0", modifier => "*0.0000305", unit => ""},
            {name => "Oxygen Sensor Voltage (Bx-Sy)", type => "word_1", modifier => "*0.000122", unit => "V"}
            ] },

            "Bank 1 - Sensor 4 (wide range O2S)" => { command => "01 27",
            result => [
            {name => "Equivalence Ratio (lambda) (Bx-Sy)", type => "word_0", modifier => "*0.0000305", unit => ""},
            {name => "Oxygen Sensor Voltage (Bx-Sy)", type => "word_1", modifier => "*0.000122", unit => "V"}
            ] },

            "Bank 2 - Sensor 1 (wide range O2S) 13" => { command => "01 28",
            result => [
            {name => "Equivalence Ratio (lambda) (Bx-Sy)", type => "word_0", modifier => "*0.0000305", unit => ""},
            {name => "Oxygen Sensor Voltage (Bx-Sy)", type => "word_1", modifier => "*0.000122", unit => "V"}
            ] },

            "Bank 2 - Sensor 2 (wide range O2S) 13" => { command => "01 29",
            result => [
            {name => "Equivalence Ratio (lambda) (Bx-Sy)", type => "word_0", modifier => "*0.0000305", unit => ""},
            {name => "Oxygen Sensor Voltage (Bx-Sy)", type => "word_1", modifier => "*0.000122", unit => "V"}
            ] },

            "Bank 2 - Sensor 3 (wide range O2S)" => { command => "01 2A",
            result => [
            {name => "Equivalence Ratio (lambda) (Bx-Sy)", type => "word_0", modifier => "*0.0000305", unit => ""},
            {name => "Oxygen Sensor Voltage (Bx-Sy)", type => "word_1", modifier => "*0.000122", unit => "V"}
            ] },

            "Bank 2 - Sensor 4 (wide range O2S)" => { command => "01 2B",
            result => [
            {name => "Equivalence Ratio (lambda) (Bx-Sy)", type => "word_0", modifier => "*0.0000305", unit => ""},
            {name => "Oxygen Sensor Voltage (Bx-Sy)", type => "word_1", modifier => "*0.000122", unit => "V"}
            ] },


            "Bank 2 - Sensor 1 (wide range O2S) 1D" => { command => "01 26",
            result => [
            {name => "Equivalence Ratio (lambda) (Bx-Sy)", type => "word_0", modifier => "*0.0000305", unit => ""},
            {name => "Oxygen Sensor Voltage (Bx-Sy)", type => "word_1", modifier => "*0.000122", unit => "V"}
            ] },

            "Bank 2 - Sensor 2 (wide range O2S) 1D" => { command => "01 27",
            result => [
            {name => "Equivalence Ratio (lambda) (Bx-Sy)", type => "word_0", modifier => "*0.0000305", unit => ""},
            {name => "Oxygen Sensor Voltage (Bx-Sy)", type => "word_1", modifier => "*0.000122", unit => "V"}
            ] },

            "Bank 3 - Sensor 1 (wide range O2S)" => { command => "01 28",
            result => [
            {name => "Equivalence Ratio (lambda) (Bx-Sy)", type => "word_0", modifier => "*0.0000305", unit => ""},
            {name => "Oxygen Sensor Voltage (Bx-Sy)", type => "word_1", modifier => "*0.000122", unit => "V"}
            ] },

            "Bank 3 - Sensor 2 (wide range O2S)" => { command => "01 29",
            result => [
            {name => "Equivalence Ratio (lambda) (Bx-Sy)", type => "word_0", modifier => "*0.0000305", unit => ""},
            {name => "Oxygen Sensor Voltage (Bx-Sy)", type => "word_1", modifier => "*0.000122", unit => "V"}
            ] },

            "Bank 4 - Sensor 1 (wide range O2S)" => { command => "01 2A",
            result => [
            {name => "Equivalence Ratio (lambda) (Bx-Sy)", type => "word_0", modifier => "*0.0000305", unit => ""},
            {name => "Oxygen Sensor Voltage (Bx-Sy)", type => "word_1", modifier => "*0.000122", unit => "V"}
            ] },

            "Bank 4 - Sensor 2 (wide range O2S)" => { command => "01 2B",
            result => [
            {name => "Equivalence Ratio (lambda) (Bx-Sy)", type => "word_0", modifier => "*0.0000305", unit => ""},
            {name => "Oxygen Sensor Voltage (Bx-Sy)", type => "word_1", modifier => "*0.000122", unit => "V"}
            ] },

            "Commanded EGR" => { command => "01 2C", result => [{type => "byte_0", modifier => "*100/255", unit => "%"}] },
            "EGR Error" => { command => "01 2D", result => [{type => "signed_byte_0", modifier => "*100/128", unit => "%"}] },
            "Commanded Evaporative Purge" => { command => "01 2E", result => [{type => "byte_0", modifier => "*100/255", unit => "%"}] },
            "Fuel Level Input" => { command => "01 2F", result => [{type => "byte_0", modifier => "*100/255", unit => "%"}] },
            "Number of warm-ups since diagnostic trouble codes cleared" => { command => "01 30", result => [{type => "byte_0", modifier => "+0", unit => ""}] },
            "Distance since diagnostic trouble codes cleared" => { command => "01 31", result => [{type => "word_0", modifier => "+0", unit => "km"}] },
            "Evap System Vapor Pressure" => { command => "01 32", result => [{type => "signed_word_0", modifier => "*0.25", unit => "Pa"}] },
            "Barometric Pressure" => { command => "01 33", result => [{type => "byte_0", modifier => "+0", unit => "kPa"}] },

            "Bank 1 - Sensor 1 (wide range O2S) current" => { command => "01 34",
            result => [
            {name => "Equivalence Ratio (lambda) (Bx-Sy)", type => "word_0", modifier => "*0.0000305", unit => ""},
            {name => "Oxygen Sensor Current (Bx-Sy)", type => "signed_word_1", modifier => "*0.000122", unit => "A"}
            ] },
            
            "Bank 1 - Sensor 2 (wide range O2S) current" => { command => "01 35",
            result => [
            {name => "Equivalence Ratio (lambda) (Bx-Sy)", type => "word_0", modifier => "*0.0000305", unit => ""},
            {name => "Oxygen Sensor Current (Bx-Sy)", type => "signed_word_1", modifier => "*0.000122", unit => "A"}
            ] },

            "Bank 1 - Sensor 3 (wide range O2S) current" => { command => "01 36",
            result => [
            {name => "Equivalence Ratio (lambda) (Bx-Sy)", type => "word_0", modifier => "*0.0000305", unit => ""},
            {name => "Oxygen Sensor Current (Bx-Sy)", type => "signed_word_1", modifier => "*0.000122", unit => "A"}
            ] },

            "Bank 1 - Sensor 4 (wide range O2S) current" => { command => "01 37",
            result => [
            {name => "Equivalence Ratio (lambda) (Bx-Sy)", type => "word_0", modifier => "*0.0000305", unit => ""},
            {name => "Oxygen Sensor Current (Bx-Sy)", type => "signed_word_1", modifier => "*0.000122", unit => "A"}
            ] },

            "Bank 2 - Sensor 1 (wide range O2S) current 13" => { command => "01 38",
            result => [
            {name => "Equivalence Ratio (lambda) (Bx-Sy)", type => "word_0", modifier => "*0.0000305", unit => ""},
            {name => "Oxygen Sensor Currente (Bx-Sy)", type => "signed_word_1", modifier => "*0.000122", unit => "A"}
            ] },

            "Bank 2 - Sensor 2 (wide range O2S) current 13" => { command => "01 39",
            result => [
            {name => "Equivalence Ratio (lambda) (Bx-Sy)", type => "word_0", modifier => "*0.0000305", unit => ""},
            {name => "Oxygen Sensor Current (Bx-Sy)", type => "signed_word_1", modifier => "*0.000122", unit => "A"}
            ] },

            "Bank 2 - Sensor 3 (wide range O2S) current" => { command => "01 3A",
            result => [
            {name => "Equivalence Ratio (lambda) (Bx-Sy)", type => "word_0", modifier => "*0.0000305", unit => ""},
            {name => "Oxygen Sensor Current (Bx-Sy)", type => "signed_word_1", modifier => "*0.000122", unit => "A"}
            ] },

            "Bank 2 - Sensor 4 (wide range O2S) current" => { command => "01 3B",
            result => [
            {name => "Equivalence Ratio (lambda) (Bx-Sy)", type => "word_0", modifier => "*0.0000305", unit => ""},
            {name => "Oxygen Sensor Current (Bx-Sy)", type => "signed_word_1", modifier => "*0.000122", unit => "A"}
            ] },


            "Bank 2 - Sensor 1 (wide range O2S) current 1D" => { command => "01 36",
            result => [
            {name => "Equivalence Ratio (lambda) (Bx-Sy)", type => "word_0", modifier => "*0.0000305", unit => ""},
            {name => "Oxygen Sensor Current (Bx-Sy)", type => "signed_word_1", modifier => "*0.000122", unit => "A"}
            ] },

            "Bank 2 - Sensor 2 (wide range O2S) current 1D" => { command => "01 37",
            result => [
            {name => "Equivalence Ratio (lambda) (Bx-Sy)", type => "word_0", modifier => "*0.0000305", unit => ""},
            {name => "Oxygen Sensor Current (Bx-Sy)", type => "signed_word_1", modifier => "*0.000122", unit => "A"}
            ] },

            "Bank 3 - Sensor 1 (wide range O2S) current" => { command => "01 38",
            result => [
            {name => "Equivalence Ratio (lambda) (Bx-Sy)", type => "word_0", modifier => "*0.0000305", unit => ""},
            {name => "Oxygen Sensor Current (Bx-Sy)", type => "signed_word_1", modifier => "*0.000122", unit => "A"}
            ] },

            "Bank 3 - Sensor 2 (wide range O2S) current" => { command => "01 39",
            result => [
            {name => "Equivalence Ratio (lambda) (Bx-Sy)", type => "word_0", modifier => "*0.0000305", unit => ""},
            {name => "Oxygen Sensor Current (Bx-Sy)", type => "signed_word_1", modifier => "*0.000122", unit => "A"}
            ] },

            "Bank 4 - Sensor 1 (wide range O2S) current" => { command => "01 3A",
            result => [
            {name => "Equivalence Ratio (lambda) (Bx-Sy)", type => "word_0", modifier => "*0.0000305", unit => ""},
            {name => "Oxygen Sensor Current (Bx-Sy)", type => "signed_word_1", modifier => "*0.000122", unit => "A"}
            ] },

            "Bank 4 - Sensor 2 (wide range O2S) current" => { command => "01 3B",
            result => [
            {name => "Equivalence Ratio (lambda) (Bx-Sy)", type => "word_0", modifier => "*0.0000305", unit => ""},
            {name => "Oxygen Sensor Current (Bx-Sy)", type => "signed_word_1", modifier => "*0.000122", unit => "A"}
            ] },
            
            "Catalyst Temperature Bank 1, Sensor 1" => { command => "01 3C", result => [{type => "word_0", modifier => "/10-40", unit => "°C"}] },
            "Catalyst Temperature Bank 1, Sensor 2" => { command => "01 3D", result => [{type => "word_0", modifier => "/10-40", unit => "°C"}] },
            "Catalyst Temperature Bank 2, Sensor 1" => { command => "01 3E", result => [{type => "word_0", modifier => "/10-40", unit => "°C"}] },
            "Catalyst Temperature Bank 2, Sensor 2" => { command => "01 3F", result => [{type => "word_0", modifier => "/10-40", unit => "°C"}] },

            "01 PIDs supported (41-60)" => { command => "01 40",
            result => [
            {name => "Monitor status this driving cycle", type => "bool_0", modifier => "&128", unit => ""},
            {name => "Control Module Voltage", type => "bool_0", modifier => "&64", unit => ""},
            {name => "Absolute Load Value", type => "bool_0", modifier => "&32", unit => ""},
            {name => "Commanded Equivalence Ratio", type => "bool_0", modifier => "&16", unit => ""},
            {name => "Relative Throttle Position", type => "bool_0", modifier => "&8", unit => ""},
            {name => "Ambiant Air Temperature", type => "bool_0", modifier => "&4", unit => ""},
            {name => "Absolute Throttle Position B", type => "bool_0", modifier => "&2", unit => ""},
            {name => "Absolute Throttle Position C", type => "bool_0", modifier => "&1", unit => ""},

            {name => "Absolute Throttle Position D", type => "bool_1", modifier => "&128", unit => ""},
            {name => "Absolute Throttle Position E", type => "bool_1", modifier => "&64", unit => ""},
            {name => "Absolute Throttle Position F", type => "bool_1", modifier => "&32", unit => ""},
            {name => "Commanded Throttle Actuator Control", type => "bool_1", modifier => "&16", unit => ""},
            {name => "Minutes run by the engine while MIL activated", type => "bool_1", modifier => "&8", unit => ""},
            {name => "Time since diagnostic trouble codes cleared", type => "bool_1", modifier => "&4", unit => ""},
            {name => "External Test Equipment Configuration Information #1", type => "bool_1", modifier => "&2", unit => ""},
            {name => "External Test Equipment Configuration Information #2", type => "bool_1", modifier => "&1", unit => ""},
            
            {name => "Type of fuel currently being utilized by the vehicle", type => "bool_2", modifier => "&128", unit => ""},
            {name => "Alcohol Fuel Percentage", type => "bool_2", modifier => "&64", unit => ""},
            {name => "Absolute Evap System Vapour Pressure", type => "bool_2", modifier => "&32", unit => ""},
            {name => "Evap System Vapour Pressure", type => "bool_2", modifier => "&16", unit => ""},
            {name => "Short Term Secondary O2 Sensor Fuel Trim - Bank 1", type => "bool_2", modifier => "&8", unit => ""},
            {name => "Long Term Secondary O2 Sensor Fuel Trim - Bank 1", type => "bool_2", modifier => "&4", unit => ""},
            {name => "Short Term Secondary O2 Sensor Fuel Trim - Bank 2", type => "bool_2", modifier => "&2", unit => ""},
            {name => "Long Term Secondary O2 Sensor Fuel Trim - Bank 2", type => "bool_2", modifier => "&1", unit => ""},
            {name => "Short Term Secondary O2 Sensor Fuel Trim - Bank 3", type => "bool_2", modifier => "&8", unit => ""},
            {name => "Long Term Secondary O2 Sensor Fuel Trim - Bank 3", type => "bool_2", modifier => "&4", unit => ""},
            {name => "Short Term Secondary O2 Sensor Fuel Trim - Bank 4", type => "bool_2", modifier => "&2", unit => ""},
            {name => "Long Term Secondary O2 Sensor Fuel Trim - Bank 4", type => "bool_2", modifier => "&1", unit => ""},
            
            {name => "Fuel Rail Pressure (absolute)", type => "bool_3", modifier => "&128", unit => ""},
            {name => "Relative Accelerator Pedal Position", type => "bool_3", modifier => "&64", unit => ""},
#            {name => "", type => "bool_3", modifier => "&32", unit => ""},
#            {name => "", type => "bool_3", modifier => "&16", unit => ""},
#            {name => "", type => "bool_3", modifier => "&8", unit => ""},
#            {name => "", type => "bool_3", modifier => "&4", unit => ""},
#            {name => "", type => "bool_3", modifier => "&2", unit => ""},
#            {name => "01 PIDs supported (61-80)", type => "bool_3", modifier => "&1", unit => ""},
            ] },

            
            "Monitor status this driving cycle" => { command => "01 41",
            result => [
            {name => "Misfire monitoring Enable Status", type => "bool_1", modifier => "&1", unit => ""},
            {name => "Fuel system monitoring Enable Status", type => "bool_1", modifier => "&2", unit => ""},
            {name => "Comprehensive component monitoring Enable Status", type => "bool_1", modifier => "&4", unit => ""},
            {name => "Misfire monitoring Completion Status", type => "bool_1", modifier => "&16", unit => ""},
            {name => "Fuel system monitoring Completion Status", type => "bool_1", modifier => "&32", unit => ""},
            {name => "Comprehensive component monitoring Completion Status", type => "bool_1", modifier => "&64", unit => ""},
            {name => "Catalyst monitoring Enable Status", type => "bool_2", modifier => "&1", unit => ""},
            {name => "Heated catalyst monitoring Enable Status", type => "bool_2", modifier => "&2", unit => ""},
            {name => "Evaporative system monitoring Enable Status", type => "bool_2", modifier => "&4", unit => ""},
            {name => "Secondary air system monitoring Enable Status", type => "bool_2", modifier => "&8", unit => ""},
            {name => "A/C system refrigerant monitoring Enable Status", type => "bool_2", modifier => "&16", unit => ""},
            {name => "Oxygen sensor monitoring Enable Status", type => "bool_2", modifier => "&32", unit => ""},
            {name => "Oxygen sensor heater monitoring Enable Status", type => "bool_2", modifier => "&64", unit => ""},
            {name => "EGR monitoring Enable Status", type => "bool_2", modifier => "&128", unit => ""},
            {name => "Catalyst monitoring Completion Status", type => "bool_3", modifier => "&1", unit => ""},
            {name => "Heated catalyst monitoring Completion Status", type => "bool_3", modifier => "&2", unit => ""},
            {name => "Evaporative system monitoring Completion Status", type => "bool_3", modifier => "&4", unit => ""},
            {name => "Secondary air system monitoring Completion Status", type => "bool_3", modifier => "&8", unit => ""},
            {name => "A/C system refrigerant monitoring Completion Status", type => "bool_3", modifier => "&16", unit => ""},
            {name => "Oxygen sensor monitoring Completion Status", type => "bool_3", modifier => "&32", unit => ""},
            {name => "Oxygen sensor heater monitoring Completion Status", type => "bool_3", modifier => "&64", unit => ""},
            {name => "EGR monitoring Completion Status", type => "bool_3", modifier => "&128", unit => ""}
            ] },
            
            "Control Module Voltage" => { command => "01 42", result => [{type => "word_0", modifier => "*0.001", unit => "V"}] },
            "Absolute Load Value" => { command => "01 43", result => [{type => "word_0", modifier => "*100/255", unit => "%"}] },
            "Commanded Equivalence Ratio" => { command => "01 44", result => [{type => "word_0", modifier => "*0.0000305", unit => ""}] },
            "Relative Throttle Position" => { command => "01 45", result => [{type => "byte_0", modifier => "*100/255", unit => "%"}] },
            "Ambiant Air Temperature" => { command => "01 46", result => [{type => "byte_0", modifier => "-40", unit => "°C"}] },
            "Absolute Throttle Position B" => { command => "01 47", result => [{type => "byte_0", modifier => "*100/255", unit => "%"}] },
            "Absolute Throttle Position C" => { command => "01 48", result => [{type => "byte_0", modifier => "*100/255", unit => "%"}] },
            "Accelerator Pedal Position D" => { command => "01 49", result => [{type => "byte_0", modifier => "*100/255", unit => "%"}] },
            "Accelerator Pedal Position E" => { command => "01 4A", result => [{type => "byte_0", modifier => "*100/255", unit => "%"}] },
            "Accelerator Pedal Position F" => { command => "01 4B", result => [{type => "byte_0", modifier => "*100/255", unit => "%"}] },
            "Commanded Throttle Actuator Control" => { command => "01 4C", result => [{type => "byte_0", modifier => "*100/255", unit => "%"}] },
            "Minutes run by the engine while MIL activated" => { command => "01 4D", result => [{type => "word_0", modifier => "+0", unit => "min"}] },
            "Time since diagnostic trouble codes cleared" => { command => "01 4E", result => [{type => "word_0", modifier => "+0", unit => "min"}] },
            "External Test Equipment Configuration Information #1" => { command => "01 4F",
            result => [
            {name => "Maximum value for Equivalence Ratio", type => "byte_0", modifier => "+0", unit => ""},
            {name => "Maximum value for Oxygen Sensor Voltage", type => "byte_1", modifier => "+0", unit => "V"},
            {name => "Maximum value for Oxygen Sensor Current", type => "byte_2", modifier => "+0", unit => "mA"},
            {name => "Maximum value for Intake Manifold Absolute Pressure", type => "byte_3", modifier => "+0", unit => "kPa"},
            ] },
            "External Test Equipment Configuration Information #2" => { command => "01 50",
            result => [
            {name => "Maximum value for Air Flow Rate from Mass Air Flow Sensor", type => "byte_0", modifier => "+0", unit => "g/s"}
            ] },

            "Type of fuel currently being utilized by the vehicle" => { command => "01 51", result => [{type => "byte_0", modifier => "+0", unit => "",
            alternatives => [
            {value => 1, meaning => "Gasoline/petrol"},
            {value => 2, meaning => "Methanol"},
            {value => 3, meaning => "Ethanol"},
            {value => 4, meaning => "Diesel"},
            {value => 5, meaning => "Liquefied Petroleum Gas (LPG)"},
            {value => 6, meaning => "Compressed Natural Gas (CNG)"},
            {value => 7, meaning => "Propane"},
            {value => 8, meaning => "Battery/electric"},
            {value => 9, meaning => "Bi-fuel vehicle using gasoline"},
            {value => 10, meaning => "Bi-fuel vehicle using methanol"},
            {value => 11, meaning => "Bi-fuel vehicle using ethanol"},
            {value => 12, meaning => "Bi-fuel vehicle using LPG"},
            {value => 13, meaning => "Bi-fuel vehicle using CNG"},
            {value => 14, meaning => "Bi-fuel vehicle using propane"},
            {value => 15, meaning => "Bi-fuel vehicle using battery"}
            ] }
            ] },
            
            "Alcohol Fuel Percentage" => { command => "01 52", result => [{type => "byte_0", modifier => "*100/255", unit => "%"}] },
            "Absolute Evap System Vapour Pressure" => { command => "01 53", result => [{type => "word_0", modifier => "/200", unit => "kPa"}] },
            "Evap System Vapour Pressure" => { command => "01 54", result => [{type => "signed_word_0", modifier => "+1", unit => "Pa"}] },
            "Short Term Secondary O2 Sensor Fuel Trim - Bank 1" => { command => "01 55", result => [{type => "signed_byte_0", modifier => "*100/128", unit => "%"}] },
            "Long Term Secondary O2 Sensor Fuel Trim - Bank 1" => { command => "01 56", result => [{type => "signed_byte_0", modifier => "*100/128", unit => "%"}] },
            "Short Term Secondary O2 Sensor Fuel Trim - Bank 2" => { command => "01 57", result => [{type => "signed_byte_0", modifier => "*100/128", unit => "%"}] },
            "Long Term Secondary O2 Sensor Fuel Trim - Bank 2" => { command => "01 58", result => [{type => "signed_byte_0", modifier => "*100/128", unit => "%"}] },

            "Short Term Secondary O2 Sensor Fuel Trim - Bank 3" => { command => "01 55", result => [{type => "signed_byte_1", modifier => "*100/128", unit => "%"}] },
            "Long Term Secondary O2 Sensor Fuel Trim - Bank 3" => { command => "01 56", result => [{type => "signed_byte_1", modifier => "*100/128", unit => "%"}] },
            "Short Term Secondary O2 Sensor Fuel Trim - Bank 4" => { command => "01 57", result => [{type => "signed_byte_1", modifier => "*100/128", unit => "%"}] },
            "Long Term Secondary O2 Sensor Fuel Trim - Bank 4" => { command => "01 58", result => [{type => "signed_byte_1", modifier => "*100/128", unit => "%"}] },

            "Fuel Rail Pressure (absolute)" => { command => "01 59", result => [{type => "word_0", modifier => "*10", unit => "kPa"}] },
            "Relative Accelerator Pedal Position" => { command => "01 5A", result => [{type => "byte_0", modifier => "*100/255", unit => "%"}] },

#            "02 PIDs supported (01-20)" => { command => "02 00",
#            result => [
#            {name => "", type => "bool_0", modifier => "&128", unit => ""},
#            {name => "", type => "bool_0", modifier => "&64", unit => ""},
#            {name => "", type => "bool_0", modifier => "&32", unit => ""},
#            {name => "", type => "bool_0", modifier => "&16", unit => ""},
#            {name => "", type => "bool_0", modifier => "&8", unit => ""},
#            {name => "", type => "bool_0", modifier => "&4", unit => ""},
#            {name => "", type => "bool_0", modifier => "&2", unit => ""},
#            {name => "", type => "bool_0", modifier => "&1", unit => ""},

#            {name => "", type => "bool_1", modifier => "&128", unit => ""},
#            {name => "", type => "bool_1", modifier => "&64", unit => ""},
#            {name => "", type => "bool_1", modifier => "&32", unit => ""},
#            {name => "", type => "bool_1", modifier => "&16", unit => ""},
#            {name => "", type => "bool_1", modifier => "&8", unit => ""},
#            {name => "", type => "bool_1", modifier => "&4", unit => ""},
#            {name => "", type => "bool_1", modifier => "&2", unit => ""},
#            {name => "", type => "bool_1", modifier => "&1", unit => ""},
            
#            {name => "", type => "bool_2", modifier => "&128", unit => ""},
#            {name => "", type => "bool_2", modifier => "&64", unit => ""},
#            {name => "", type => "bool_2", modifier => "&32", unit => ""},
#            {name => "", type => "bool_2", modifier => "&16", unit => ""},
#            {name => "", type => "bool_2", modifier => "&8", unit => ""},
#            {name => "", type => "bool_2", modifier => "&4", unit => ""},
#            {name => "", type => "bool_2", modifier => "&2", unit => ""},
#            {name => "", type => "bool_2", modifier => "&1", unit => ""},
            
#            {name => "", type => "bool_3", modifier => "&128", unit => ""},
#            {name => "", type => "bool_3", modifier => "&64", unit => ""},
#            {name => "", type => "bool_3", modifier => "&32", unit => ""},
#            {name => "", type => "bool_3", modifier => "&16", unit => ""},
#            {name => "", type => "bool_3", modifier => "&8", unit => ""},
#            {name => "", type => "bool_3", modifier => "&4", unit => ""},
#            {name => "", type => "bool_3", modifier => "&2", unit => ""},
#            {name => "02 PIDs supported (41-60)", type => "bool_3", modifier => "&1", unit => ""},
#            ] },
            

#            "03 PIDs supported (01-20)" => { command => "03 00",
#            result => [
#            {name => "", type => "bool_0", modifier => "&128", unit => ""},
#            {name => "", type => "bool_0", modifier => "&64", unit => ""},
#            {name => "", type => "bool_0", modifier => "&32", unit => ""},
#            {name => "", type => "bool_0", modifier => "&16", unit => ""},
#            {name => "", type => "bool_0", modifier => "&8", unit => ""},
#            {name => "", type => "bool_0", modifier => "&4", unit => ""},
#            {name => "", type => "bool_0", modifier => "&2", unit => ""},
#            {name => "", type => "bool_0", modifier => "&1", unit => ""},

#            {name => "", type => "bool_1", modifier => "&128", unit => ""},
#            {name => "", type => "bool_1", modifier => "&64", unit => ""},
#            {name => "", type => "bool_1", modifier => "&32", unit => ""},
#            {name => "", type => "bool_1", modifier => "&16", unit => ""},
#            {name => "", type => "bool_1", modifier => "&8", unit => ""},
#            {name => "", type => "bool_1", modifier => "&4", unit => ""},
#            {name => "", type => "bool_1", modifier => "&2", unit => ""},
#            {name => "", type => "bool_1", modifier => "&1", unit => ""},
            
#            {name => "", type => "bool_2", modifier => "&128", unit => ""},
#            {name => "", type => "bool_2", modifier => "&64", unit => ""},
#            {name => "", type => "bool_2", modifier => "&32", unit => ""},
#            {name => "", type => "bool_2", modifier => "&16", unit => ""},
#            {name => "", type => "bool_2", modifier => "&8", unit => ""},
#            {name => "", type => "bool_2", modifier => "&4", unit => ""},
#            {name => "", type => "bool_2", modifier => "&2", unit => ""},
#            {name => "", type => "bool_2", modifier => "&1", unit => ""},
            
#            {name => "", type => "bool_3", modifier => "&128", unit => ""},
#            {name => "", type => "bool_3", modifier => "&64", unit => ""},
#            {name => "", type => "bool_3", modifier => "&32", unit => ""},
#            {name => "", type => "bool_3", modifier => "&16", unit => ""},
#            {name => "", type => "bool_3", modifier => "&8", unit => ""},
#            {name => "", type => "bool_3", modifier => "&4", unit => ""},
#            {name => "", type => "bool_3", modifier => "&2", unit => ""},
#            {name => "03 PIDs supported (41-60)", type => "bool_3", modifier => "&1", unit => ""},
#            ] },
            

            "05 MIDs supported (01-20)" => { command => "05 00", available => 1,
            result => [
            {name => "Rich to lean sensor threshold voltage", type => "bool_0", modifier => "&128", unit => ""},
            {name => "Lean to rich sensor threshold voltage", type => "bool_0", modifier => "&64", unit => ""},
            {name => "Low sensor voltage for switch time calculation", type => "bool_0", modifier => "&32", unit => ""},
            {name => "High sensor voltage for switch time calculation", type => "bool_0", modifier => "&16", unit => ""},
            {name => "Rich to lean sensor switch time", type => "bool_0", modifier => "&8", unit => ""},
            {name => "Lean to rich sensor switch time", type => "bool_0", modifier => "&4", unit => ""},
            {name => "Minimum sensor voltage for test cycle", type => "bool_0", modifier => "&2", unit => ""},
            {name => "Maximum sensor voltage for test cycle", type => "bool_0", modifier => "&1", unit => ""},

            {name => "Time between sensor transitions", type => "bool_1", modifier => "&128", unit => ""},
            {name => "Sensor period", type => "bool_1", modifier => "&64", unit => ""},
#            {name => "", type => "bool_1", modifier => "&32", unit => ""},
#            {name => "", type => "bool_1", modifier => "&16", unit => ""},
#            {name => "", type => "bool_1", modifier => "&8", unit => ""},
#            {name => "", type => "bool_1", modifier => "&4", unit => ""},
#            {name => "", type => "bool_1", modifier => "&2", unit => ""},
#            {name => "", type => "bool_1", modifier => "&1", unit => ""},
            
#            {name => "", type => "bool_2", modifier => "&128", unit => ""},
#            {name => "", type => "bool_2", modifier => "&64", unit => ""},
#            {name => "", type => "bool_2", modifier => "&32", unit => ""},
#            {name => "", type => "bool_2", modifier => "&16", unit => ""},
#            {name => "", type => "bool_2", modifier => "&8", unit => ""},
#            {name => "", type => "bool_2", modifier => "&4", unit => ""},
#            {name => "", type => "bool_2", modifier => "&2", unit => ""},
#            {name => "", type => "bool_2", modifier => "&1", unit => ""},
            
#            {name => "", type => "bool_3", modifier => "&128", unit => ""},
#            {name => "", type => "bool_3", modifier => "&64", unit => ""},
#            {name => "", type => "bool_3", modifier => "&32", unit => ""},
#            {name => "", type => "bool_3", modifier => "&16", unit => ""},
#            {name => "", type => "bool_3", modifier => "&8", unit => ""},
#            {name => "", type => "bool_3", modifier => "&4", unit => ""},
#            {name => "", type => "bool_3", modifier => "&2", unit => ""},
#            {name => "05 PIDs supported (41-60)", type => "bool_3", modifier => "&1", unit => ""},
            ] },
            
            "Rich to lean sensor threshold voltage" => { command => "05 01", result => [{type => "byte_0", modifier => "*0.005", unit => "V"}] },
            "Lean to rich sensor threshold voltage" => { command => "05 02", result => [{type => "byte_0", modifier => "*0.005", unit => "V"}] },
            "Low sensor voltage for switch time calculation" => { command => "05 03", result => [{type => "byte_0", modifier => "*0.005", unit => "V"}] },
            "High sensor voltage for switch time calculation" => { command => "05 04", result => [{type => "byte_0", modifier => "*0.005", unit => "V"}] },
            "Rich to lean sensor switch time" => { command => "05 05", result => [{type => "byte_0", modifier => "*0.004", unit => "s"}] },
            "Lean to rich sensor switch time" => { command => "05 06", result => [{type => "byte_0", modifier => "*0.004", unit => "s"}] },
            "Minimum sensor voltage for test cycle" => { command => "05 07", result => [{type => "byte_0", modifier => "*0.005", unit => "V"}] },
            "Maximum sensor voltage for test cycle" => { command => "05 08", result => [{type => "byte_0", modifier => "*0.005", unit => "V"}] },
            "Time between sensor transitions" => { command => "05 09", result => [{type => "byte_0", modifier => "*0.04", unit => "s"}] },
            "Sensor period" => { command => "05 0A", result => [{type => "byte_0", modifier => "*0.04", unit => "s"}] },




            "09 PIDs supported (01-20)" => { command => "09 00", available => 1,
            result => [
            {name => "MessageCount VIN", type => "bool_0", modifier => "&128", unit => ""},
            {name => "Vehicle Identification Number", type => "bool_0", modifier => "&64", unit => ""},
            {name => "MessageCount CALID", type => "bool_0", modifier => "&32", unit => ""},
            {name => "Calibration Identifications", type => "bool_0", modifier => "&16", unit => ""},
            {name => "MessageCount CVN", type => "bool_0", modifier => "&8", unit => ""},
            {name => "Calibration Verification Numbers", type => "bool_0", modifier => "&4", unit => ""},
            {name => "MessageCount IPT", type => "bool_0", modifier => "&2", unit => ""},
            {name => "In-use Performance Tracking", type => "bool_0", modifier => "&1", unit => ""},

            {name => "MessageCount ECUNAME", type => "bool_1", modifier => "&128", unit => ""},
            {name => "ECUNAME", type => "bool_1", modifier => "&64", unit => ""},
#            {name => "", type => "bool_1", modifier => "&32", unit => ""},
#            {name => "", type => "bool_1", modifier => "&16", unit => ""},
#            {name => "", type => "bool_1", modifier => "&8", unit => ""},
#            {name => "", type => "bool_1", modifier => "&4", unit => ""},
#            {name => "", type => "bool_1", modifier => "&2", unit => ""},
#            {name => "", type => "bool_1", modifier => "&1", unit => ""},
            
#            {name => "", type => "bool_2", modifier => "&128", unit => ""},
#            {name => "", type => "bool_2", modifier => "&64", unit => ""},
#            {name => "", type => "bool_2", modifier => "&32", unit => ""},
#            {name => "", type => "bool_2", modifier => "&16", unit => ""},
#            {name => "", type => "bool_2", modifier => "&8", unit => ""},
#            {name => "", type => "bool_2", modifier => "&4", unit => ""},
#            {name => "", type => "bool_2", modifier => "&2", unit => ""},
#            {name => "", type => "bool_2", modifier => "&1", unit => ""},
            
#            {name => "", type => "bool_3", modifier => "&128", unit => ""},
#            {name => "", type => "bool_3", modifier => "&64", unit => ""},
#            {name => "", type => "bool_3", modifier => "&32", unit => ""},
#            {name => "", type => "bool_3", modifier => "&16", unit => ""},
#            {name => "", type => "bool_3", modifier => "&8", unit => ""},
#            {name => "", type => "bool_3", modifier => "&4", unit => ""},
#            {name => "", type => "bool_3", modifier => "&2", unit => ""},
#            {name => "09 PIDs supported (41-60)", type => "bool_3", modifier => "&1", unit => ""},
            ] },            

            "MessageCount VIN" => { command => "09 01", result => [{type => "byte_0", modifier => "+0", unit => ""}] },
            "Vehicle Identification Number" => { command => "09 02", result => [{type => "string_1", modifier => "", unit => ""}] },
            "MessageCount CALID" => { command => "09 03", result => [{type => "byte_0", modifier => "+0", unit => ""}] },
            "Calibration Identifications" => { command => "09 04", result => [{type => "string_1", modifier => "", unit => ""}] },
            "MessageCount CVN" => { command => "09 05", result => [{type => "byte_0", modifier => "+0", unit => ""}] },
            "Calibration Verification Numbers" => { command => "09 06", result => [{type => "dword_0", modifier => "+0", unit => ""}] },
            "MessageCount IPT" => { command => "09 07", result => [{type => "byte_0", modifier => "+0", unit => ""}] },
            "In-use Performance Tracking" => { command => "09 08",
            result => [
            {name => "OBD Monitoring Conditions Encountered Counts", type => "word_0", modifier => "+0", unit => ""},
            {name => "Ignition Counter", type => "word_1", modifier => "+0", unit => ""},
            {name => "Catalyst Monitor Completion Counts Bank 1", type => "word_2", modifier => "+0", unit => ""},
            {name => "Catalyst Monitor Conditions Encountered Counts Bank 1", type => "word_3", modifier => "+0", unit => ""},
            {name => "Catalyst Monitor Completion Counts Bank 2", type => "word_4", modifier => "+0", unit => ""},
            {name => "Catalyst Monitor Conditions Encountered Counts Bank 2", type => "word_5", modifier => "+0", unit => ""},
            {name => "O2 Sensor Monitor Completion Counts Bank 1", type => "word_6", modifier => "+0", unit => ""},
            {name => "O2 Sensor Monitor Conditions Encountered Counts Bank 1", type => "word_7", modifier => "+0", unit => ""},
            {name => "O2 Sensor Monitor Completion Counts Bank 2", type => "word_8", modifier => "+0", unit => ""},
            {name => "O2 Sensor Monitor Conditions Encountered Counts Bank 2", type => "word_9", modifier => "+0", unit => ""},
            {name => "EGR Monitor Completion Condition Counts", type => "word_10", modifier => "+0", unit => ""},
            {name => "EGR Monitor Conditions Encountered Counts", type => "word_11", modifier => "+0", unit => ""},
            {name => "AIR Monitor Completion Condition Counts (Secondary Air)", type => "word_12", modifier => "+0", unit => ""},
            {name => "AIR Monitor Conditions Encountered Counts (Secondary Air)", type => "word_13", modifier => "+0", unit => ""},
            {name => "EVAP Monitor Completion Condition Counts", type => "word_14", modifier => "+0", unit => ""},
            {name => "EVAP Monitor Conditions Encountered Counts", type => "word_15", modifier => "+0", unit => ""}
            ] },
            "MessageCount ECUNAME" => { command => "09 09", result => [{type => "byte_0", modifier => "+0", unit => ""}] },
            "ECUNAME" => { command => "09 0A", result => [{type => "string_0", modifier => "", unit => ""}] },
            
            };


	if (defined($replay_filename))
	{
    $self->Replay($replay_filename);

#    $self->ShowReadableValues();
#print Dumper($self->{'get'});
	}
  else
  {
    $self->OpenPort($port_name);
    $self->ConfigureDevice();
    $self->FindAvailableCommands();
  }

	return $self;
}


#*****************************************************************

# Try to find an ELM module on a COM port.
# If a $port_name is supplied, start with that one and work upwards.

sub OpenPort
{
  my ($self, $port_name) = @_;
  my $quiet = 0;
  my $port = -1;
  my $port_count = 0;

  if (!defined($port_name) || $port_name eq "")
  {
    if ($^O eq "MSWin32")
    {
      $port_name = "COM1";
    }
    else
    {
      $port_name = "/dev/ttyUSB0";
    }
  }

 	my $port_number = $port_name;
	$port_number =~ s/[^0-9]//g;  # Strip everything that isn't numeric

	my $port_text = $port_name;
	$port_text =~ s/[0-9]//g;     # Strip everything that is numeric

  do
  {
    $port_name = $port_text.$port_number;
    
    if ($^O eq "MSWin32")
    {
      $port = Win32::SerialPort->new ($port_name);
    }
    else
    {
      $port = Device::SerialPort->new($port_name, $quiet);
    }

    if (defined($port))
    {
      $port->user_msg(1); 	    # misc. warnings
      $port->error_msg(1); 	    # hardware and data errors

      $port->baudrate(38400);
      $port->parity("none");
      $port->parity_enable(0);  # for any parity except "none"
      $port->databits(8);
      $port->stopbits(1);
      $port->handshake('none');

      $port->write_settings;

      $self->{'port'} = $port;
      $self->Command("AT Z");   # Reset Device
      foreach (@{$self->{'response'}})
      {
        if (substr($_, 0, 5) eq "ELM32")  # Allow 328 & 329 as well
        {
          $self->{'ELM_type'} = substr($_, 0, 6);
        }
      }
      if ($self->{'ELM_type'} eq "NONE" && $self->{'debug_level'} > 0)
      {
        print "Can't find an ELM module on $port_name\n";
      }
    }
    else
    {
      if ($self->{'debug_level'} > 0)
      {
        print "Can't open $port_name: $!\n";
      }
      $self->{'port'} = -1;
    }

    $port_number++;
    $port_count++;
  } until(($self->{'port'} != -1 && $self->{'ELM_type'} ne "NONE") || $port_count > $max_ports_to_search);

  if ($self->{'ELM_type'} eq "NONE")
  {
    $self->{'port'} = -1;
    die "Couldn't find an ELM module!\n"; 
  }
}


#*****************************************************************

=head2 PortOK

Returns 1 if the serial port and ELM module are working or 0 if no ELM device could be connected to.

	$obd->PortOK();
=cut

sub PortOK
{
	my ($self) = @_;

  if ($self->{'port'} == -1)
  {
    if ($self->{'debug_level'} > 0) { print "Serial port not initialised!\n"; }
    return 0;
  }
  else
  {
    return 1;
  }
}


#*****************************************************************

# Set up the ELM module to return data in the required form

sub ConfigureDevice
{
	my ($self) = @_;

  if ($self->{'debug_level'} > 0) { print "ConfigureDevice\n"; }
  
  if ($self->PortOK)
  {
    $self->Command("AT E0");   # Turn echo off

    $self->Command("AT L0");   # Turn linefeeds off

    $self->Command("AT SP 0");  # Set Protocol to auto

    $self->Command("AT DPN");  # Display Protocol number

    $self->Command("AT DP");  # Display Protocol

    $self->Command("AT H1");  # Turn headers on

    $self->Command("01 00");  # Prepare to talk and get available PID's
  }
}


#*****************************************************************

# Query the ECU to find out which commands are supported and
# annotate the value entries in the 'get' structure with the 
# 'available' flag set to 0 (not supported) or 1 (supported).

sub FindAvailableCommands
{
	my ($self) = @_;

  my $next_command = "01 PIDs supported (01-20)";

  if ($self->{'debug_level'} > 0) { print "FindAvailableCommands:\n"; }

  do
  {
    $next_command = $self->ProcessAvailableCommands($next_command);
  } while (defined($next_command));

  $next_command = "05 MIDs supported (01-20)";
  do
  {
    $next_command = $self->ProcessAvailableCommands($next_command);
  } while (defined($next_command));

  $next_command = "09 PIDs supported (01-20)";
  do
  {
    $next_command = $self->ProcessAvailableCommands($next_command);
  } while (defined($next_command));

}


#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

# Query the ECU to find out which commands are supported and
# annotate the value entries in the 'get' structure with the 
# 'available' flag set to 0 (not supported) or 1 (supported).

sub ProcessAvailableCommands
{
	my ($self, $command) = @_;

  my $next_command = undef;
  
  if ($self->{'debug_level'} > 0) { print "~ProcessAvailableCommands: $command\n"; }

  my $response = $self->Read($command);

  if ($self->{'debug_level'} > 1) { print "$command\n"; }

  if ($response->{'status'} == 0)
  {
    foreach my $result (@{$response->{'results'}})
    {
      $self->{'get'}->{$result->{'name'}}->{'available'} = $result->{'value'};
      if ($self->{'debug_level'} > 1) { print "$result->{'address'} - $result->{'name'}: $result->{'value'} $result->{'unit'}\n"; }

      if (substr($result->{'name'}, 4, 15) eq "IDs supported (" && $result->{'value'} == 1)
      {
        $next_command = $result->{'name'};
      }
      elsif( $result->{'value'} == 1 &&
						((substr($result->{'name'}, -2, 2) eq "13" && $self->{'get'}->{'Location of oxygen sensors 13'}->{'available'} == 1)
					|| (substr($result->{'name'}, -2, 2) eq "1D" && $self->{'get'}->{'Location of oxygen sensors 1D'}->{'available'} == 1))
					 )
      {
        my $new_name = substr($result->{'name'}, 0, length($result->{'name'})-3);
        
				if ($self->{'debug_level'} > 2)
				{
					print "Old name: >$result->{'name'}<\n";
					print "New name: >$new_name<\n";
				}
				
        $self->{'get'}->{$new_name} = delete($self->{'get'}->{$result->{'name'}});
				if ($self->{'debug_level'} > 3)
				{
					print Dumper($self->{'get'}->{$new_name});
				}
      }
    }
  }

  return $next_command;
}


#*****************************************************************

=head2 ShowReadableValues

Displays the list of values that can be read from the ELM/ECU.

	$obd->ShowReadableValues();
=cut

sub ShowReadableValues
{
	my ($self) = @_;

  print "Values that the Read and Show functions can fetch from this vehicle:\n\n";

  while( my ($parameter_name, $definition) = each %{$self->{'get'}} )
  {
  	if (exists($definition->{'available'}) && $definition->{'available'} == 1)
  	{
		  print "Value: \"$parameter_name\"\n";
		  print "Returns:\n";
		  foreach my $result (@{$definition->{'result'}})
		  {
		    if (exists($result->{'name'}))
		    {
		      print "$result->{'name'}";
		    }
		    else
		    {
		      print "$parameter_name";
		    }
		    if (exists($result->{'unit'}) && $result->{'unit'} ne "")
		    {
		      print " ($result->{'unit'})";
		    }
		    elsif (substr($result->{'type'}, 0, 4) eq "bool")
		    {
		      print " (0 or 1)";
		    }
		    print "\n";
		  }
		  print "\n";
		}
  }
}


#*****************************************************************

# Process a file containing debugging output and replay the commands
# through the module.

sub Replay
{
  my ($self, $replay_filename) = @_;
  $self->{'replay_file'} = 1;

  my $replay_command="";

  # Open the file containing the command and response data
  open (REPLAYFILE, $replay_filename);

  my $get_command = 0;
  my $seek_response = 1;
  my $read_response = 2;
  my $get_result = 4;
  
  my $replay_state = $get_command; 

  while (<REPLAYFILE>)
  { 
    # Iterate through issuing commands and parsing responses
    $_ =~ s/\r?\n//g;   # Strip all carriage returns

    if ($replay_state == $get_command)
    {
      if (substr($_, 0, 1) eq "~")
      {
        $_ = substr($_, 1);
        my @linepart = split(":", $_);
        foreach my $part (@linepart)
        {
          $part =~ s/^\s+|\s+$//g; # Strip unwanted whitespace
        }

        $replay_command = '$self->'.$linepart[0]."(\"";
        my $number_of_parameters = scalar(@linepart);
        for (my $index=1; $index<$number_of_parameters; $index++)
        {
          $replay_command .= $linepart[$index];
          if ($index < ($number_of_parameters-1))
          {
            $replay_command .= ",";
          } 
        }				
        $replay_command .= "\");";
        
        $replay_state = $seek_response;
      }
    }
    elsif ($replay_state == $seek_response)
    {
      if ($_ eq "Response")
      {
        $replay_state = $read_response;
      }
    }
    elsif ($replay_state == $read_response)
    {
      if ($_ eq "End of response")
      {
        # Execute the command we just got the response to...
        my $status = 0;
        $status = eval($replay_command);
        if ($self->{'debug_level'} > 1)
        {
          print "Status: $status\n";
          print Dumper($status);
        }
        $replay_state = $get_command;
      }
      else
      {
        if (length($_) > 0)
        {
          push @{$self->{'replay_response'}}, $_;
        }
      }
    }
  }
  close (REPLAYFILE);
}


#*****************************************************************

=head2 Show

When passed the name of an OBD value (e.g. "Engine RPM") in $value_name,
it displays the address of the ECU which returned the value, the name of
the value which was read or the name of one of many parameters returned 
followed by the value and the name of any unit associated with the value.

If an error occurred, a message will be displayed instead.

	$obd->Show($value_name);

This function calls the 'Read' function.
=cut
sub Show
{
	my ($self, $name) = @_;

  if ($self->{'debug_level'} > 0) { print "~Show: $name\n"; }
  
  my $response = $self->Read($name);

  print "$name:\n";

  if ($response->{'status'} == 0)
  {
    foreach my $result (@{$response->{'results'}})
    {
      print "$result->{'address'} - $result->{'name'}: $result->{'value'} $result->{'unit'}\n"
    }
  }
  else
  {
    print "$response->{'status_message'}\n";
  }
  print "\n";

  return $response;
}


#*****************************************************************

=head2 Read

When passed the name of an OBD value (e.g. "Engine RPM") in $value_name,
it returns a reference to a structure containing a status flag and any
responses:

	my $response = $obd->Read($value_name);

	if ($response->{'status'} == 0)
	{
		foreach my $result (@{$response->{'results'}})
		{
			print "$result->{'address'} - $result->{'name'}: $result->{'value'} $result->{'unit'}\n"
		}
	}

In the example above, $result->{'address'} contains the address of the
ECU which returned the value, $result->{'name'} contains the name of
the value which was read or the name of one of many parameters returned.
$result->{'value'} and $result->{'unit'} contain the value and the name
of any unit associated with the value.

Error conditions are indicated by a non-zero value of $response->{'status'}
and an error message in $response->{'status_message'} (the default is
'ok' when there is no error).
=cut

sub Read
{
	my ($self, $name) = @_;

	my $status = 0;
	
	my $results=();
	
	my $id;
	my $address;
	my $value;

	$results->{'status'} = 0;
	$results->{'status_message'} = "ok";

  if ($self->{'debug_level'} > 0) { print "~Read: $name\n"; }

  if (exists($self->{'get'}->{$name}))
  {
		$status = $self->Command($self->{'get'}->{$name}->{'command'});

    if ($status != 0)
    {
      $results->{'status'} = -2; # No response
      $results->{'status_message'} = "No data returned for: $name";
      return $results;
    }

		foreach my $result (@{$self->{'get'}->{$name}->{'result'}})
		{
      if ($self->{'debug_level'} > 2) { print Dumper($result); }
			if (exists($result->{'name'}))
			{
				$id = $result->{'name'};  
			}
			else
			{
				$id = $name;
			}

			# Parse the result type...
			my @type = split('_', , $result->{'type'});
			
			my $index = 0;
			my $sign = 0;
			if ($type[0] eq "signed")
			{
				$sign = 1;
				$index = 1;
			}

      if ($type[$index] eq "AT")
      {
        $value = ${$self->{'response'}}[0];

        if (defined($result->{'modifier'}))
        {
          my $statement = '$value' . $result->{'modifier'};
          eval( $statement ); # Allow the use of a regex if required
          if ($self->{'debug_level'} > 2) { print "Statement: $statement\n"; }
        }
        
        if ($self->{'debug_level'} > 1) { print "$id, $value$result->{'unit'}\n"; }
        push @{$results->{'results'}},{name => $id, value => $value, unit => $result->{'unit'}, address => "ELM327"};
      }
      else
      {
        $value = $self->GetResult($type[$index], $type[$index+1]);
        if ($self->{'debug_level'} > 2) { print "GetResult value: $value\n"; }

        my $iteration = 0;
        foreach $value (@{$self->{'command_results'}})
        {
          if ($self->{'debug_level'} > 2) { print "Raw value: $value\n"; }
          
          if ($type[$index] eq "byte" && $type[0] eq "signed")
          {
            my $byte_sign_mask = 128;
            my $byte_value_mask = 127;
            
            $value = ($value & $byte_value_mask) - ($value & $byte_sign_mask);  	
          }
          elsif ($type[$index] eq "word" && $type[0] eq "signed")
          {
            my $word_sign_mask = 32768;
            my $word_value_mask = 32767;
            
            $value = ($value & $word_value_mask) - ($value & $word_sign_mask);  	
          }
          
          if ($type[$index] ne "string")
          {
            if ($self->{'debug_level'} > 2) { print "$value $result->{'modifier'}\n"; }
            $value = eval( "$value $result->{'modifier'}" );
            if ($self->{'debug_level'} > 2) { print "$value\n"; }
            
          }
          else
          {
            my $statement = '$value' . $result->{'modifier'};
            eval( $statement ); # Allow the use of a regex if required
            if ($self->{'debug_level'} > 2) { print "Statement: $statement\n"; }
            
          }
          
          if ($type[$index] eq "bool")
          {
            if ($value != 0) { $value = 1; }			
          }

          if (exists($result->{'alternatives'}))
          {
            foreach my $alternative (@{$result->{'alternatives'}})
            {
              if ($alternative->{'value'} == $value)
              {
                $value = $alternative->{'meaning'};
                last;
              }
            }
          }
          if ($self->{'debug_level'} > 2) { print "$id, $value, $result->{'unit'}\n"; }
          push @{$results->{'results'}},{name => $id, value => $value, unit => $result->{'unit'}, address => ${$self->{'command_addresses'}}[$iteration]};
          $iteration++;
        }
        $self->{'command_results'} = [];
      }
		}
	}
	else
	{
    if ($self->{'debug_level'} > 2) { print "Unrecognised name: 	$name\n"; }
		$results->{'status'} = -1;
		$results->{'status_message'} = "Unrecognised name: $name";
	}
	
  return $results;
}


#*****************************************************************

# Send a command to the ELM, read any response and decode it if
# was an ECU command.

sub Command
{
  my ($self, $command) = @_;

  my $status = 0;

  if ($self->{'debug_level'} > 0) { print "~Command: $command\n"; }
  my @command_parts = split(' ', $command);

  $status = $self->WriteCommand("$command");

  $status = $self->ReadResponse();

  if ($command_parts[0] ne "AT" && $status == 0)
  {
    $self->{'last_command'} = hex($command_parts[0]);
    if ($has_sub_command[$self->{'last_command'}])
    {
      $self->{'last_sub_command'} = hex($command_parts[1]);
    }
    else
    {
      $self->{'last_sub_command'} = 0;
    }
      
    $self->DecodeResponse();
  }

  return $status;
}


#*****************************************************************

# Write a command to the serial port

sub WriteCommand
{
  my ($self, $command) = @_;

  my $status = 0;

  if ($self->{'debug_level'} > 0) { print "~WriteCommand: $command\n"; }

  if ($self->{'replay_file'} == 0 && $self->PortOK)
  {
    $command .= "$cr$lf";
    $self->{'port'}->write("$command");
  }

  return $status;
}

#*****************************************************************

# Read the ELM's response from the serial port and put each line 
# into the $self->{'response'} array.

sub ReadResponse
{
  my ($self) = @_;
   
  my $bytes_to_read = 1;
  my $count_in = 0;
  my $string_in = "";
  my $status = 0;
  my $timeout = 4;  # Command 01 04 failed when timeout was 2
  my $line = "";
  $self->{'response'} = ();
  $self->{'response_length'} = 0;

  if ($self->{'debug_level'} > 1) { print "ReadResponse\n"; }

  if ($self->{'replay_file'} == 0 && $self->PortOK)
  {
    do
    {
      ($count_in, $string_in) = $self->{'port'}->read($bytes_to_read);
      if ($count_in == $bytes_to_read && $string_in ne $null)
      {
        $line .= $string_in;
        $self->{'response_length'}++;
      }
      else
      {
        sleep 1;
        $timeout--;
      }
    } while ($count_in == 0 && $timeout>0);

    do
    {
      ($count_in, $string_in) = $self->{'port'}->read($bytes_to_read);
      if ($count_in == $bytes_to_read)
      {
        if ($string_in ne ">" && $string_in ne $null)
        {
          if ($string_in eq $cr)
          {
            if ($line ne "")
            {
              push @{$self->{'response'}}, $line;
              $line = "";
            }
          }
          else
          {
            $line .= $string_in;
            $self->{'response_length'}++;
          }
        }
      }
    } while ($count_in == $bytes_to_read);
  }
  else
  {
    # Load in the lines of the response from the replay file
    while (scalar(@{$self->{'replay_response'}}) > 0)
    {
      $line = shift @{$self->{'replay_response'}};
      push @{$self->{'response'}}, $line;
      $self->{'response_length'} += length($line);
    }
  }

  if ($self->{'response_length'} == 0)
  {
    $status = 1;
  }
  elsif ($self->{'response'}[0] eq "NO DATA")
  {
  	$status = 2;
  }

  if ($self->{'debug_level'} > 0)
  {
    print "Response\n";
    foreach (@{$self->{'response'}})
    {
      print "$_\n";
    }
    print "End of response\n";
  }

  return $status;
}


#*****************************************************************

# Decode the ECU response (in the $self->{'response'} array) and
# write the result to the $self->{'results'} structure.

sub DecodeResponse
{
  my ($self) = @_;

  my $results = {};
  my $command_mask = 63;
  my $line_number = 0;
  my $result_string;

  $self->{'command_addresses'} = [];
  $self->{'command_results'} = [];
  $self->{'number_of_results'} = 0;
   
  if ($self->{'debug_level'} > 1)
  {
    print "\nDecodeResponse\n";
    print "Lines: ".scalar(@{$self->{'response'}})."\n";
  }

  for (keys %{$self->{'results'}})
  {
    delete $self->{'results'}->{$_};
  }

  
  foreach (@{$self->{'response'}})
  {
    my @line_parts = split(' ', $_);
    if (scalar(@line_parts) > 2)
    {
      my $address = shift @line_parts;
      if (length($address) < 3)
      {
        # Not CAN ($address contains the priority byte)
        $self->{'results'}->{$address}->{'format'} = "Other";
        $line_number++;
        my $recipient_address = shift @line_parts;     
        $address = shift @line_parts; # Transmitter address    
        $self->{'results'}->{$address}->{'command'} = (hex(shift @line_parts) & $command_mask);
        if ($self->{'results'}->{$address}->{'command'} == 4) { push(@line_parts, "00"); }
        if ($self->{'results'}->{$address}->{'command'} == $command_mask)
        {
          $self->{'results'}->{$address}->{'command'} = (hex(shift @line_parts));
        }
        if ($has_sub_command[$self->{'results'}->{$address}->{'command'}])
        {     
          $self->{'results'}->{$address}->{'sub_command'} = hex(shift @line_parts);
        }
        else
        {
          $self->{'results'}->{$address}->{'sub_command'} = 0;
        }
        if (exists($self->{'results'}->{$address}->{'response_length'}))
        {
          $self->{'results'}->{$address}->{'response_length'} += hex(scalar(@line_parts));
        }
        else
        {
          $self->{'results'}->{$address}->{'response_length'} = hex(scalar(@line_parts));
        }
      }
      else
      {
        # CAN
        $self->{'results'}->{$address}->{'format'} = "CAN";
        $line_number = hex(shift @line_parts);
    
        if ($line_number <= 16)
        {
          if ($line_number < 16)
          {
            $self->{'results'}->{$address}->{'response_length'} = $line_number;
          }
          else
          {
            $self->{'results'}->{$address}->{'response_length'} = hex(shift @line_parts);     
          }      
          $self->{'results'}->{$address}->{'command'} = (hex(shift @line_parts) & $command_mask);     
          if ($self->{'results'}->{$address}->{'command'} == 4) { push(@line_parts, "00"); }
          if ($self->{'results'}->{$address}->{'command'} == $command_mask)
          {
            $self->{'results'}->{$address}->{'command'} = (hex(shift @line_parts));
          }
          if ($has_sub_command[$self->{'results'}->{$address}->{'command'}])
          {     
            $self->{'results'}->{$address}->{'sub_command'} = hex(shift @line_parts);
          }
          else
          {
            $self->{'results'}->{$address}->{'sub_command'} = 0;
            if ($self->{'results'}->{$address}->{'command'} == 3)
            {
              $self->{'results'}->{$address}->{'number_of_DTCs'} = hex(shift @line_parts);
            }
          }
        }     
      }  
      $results->{$address}->{$line_number} = join(" ", @line_parts);
    }
  }

  if ($self->{'debug_level'} > 1)
  {
    print "Decoded results:\n";  
    print Dumper($results);
  }

  foreach my $address (sort keys %{$results})
  {
    $result_string = "";
    foreach my $line (sort keys %{$results->{$address}})
    {
      $result_string .= "$results->{$address}->{$line} "; 
    }
    @{$self->{'results'}->{$address}->{'result'}} = split(' ', $result_string);


    #Now turn the hex byte strings into numbers...
    foreach (@{$self->{'results'}->{$address}->{'result'}})
    {
      $_ = hex($_);
    }
  }

  if ($self->{'debug_level'} > 1)
  {
    print "\nFully decoded results:\n";  
    print Dumper($self->{'results'});
  }
  
}


#*****************************************************************

# Returns a value from the last set of results from the ELM/ECU

# $type can be one of the following:
# bool (1 bit), byte (8 bit), word (16 bit), dword (32 bit) or string.
# $number is the zero-based index into the array of results and takes
# the type into account such that $number=0 returns the first byte,
# word or dword and $number=1, returns the second.
# Booleans are treated the same as bytes and require individual bits
# to be extracted separately.
# For strings, $number is the offset of the start of the string.

sub GetResult
{
	my ($self, $type, $number) = @_;

  my $result;
  $self->{'number_of_results'} = 0;

  if (!defined($number)) { $number = 0; }
  
  if ($self->{'debug_level'} > 1) { print "GetResult: $type: $number\n"; }

  foreach my $address (sort keys %{$self->{'results'}})
  {
    if ($self->{'results'}->{$address}->{'command'} == $self->{'last_command'} && 
    $self->{'results'}->{$address}->{'sub_command'} == $self->{'last_sub_command'})
    {
      if ($type eq "bool")
      {
        $result = ${$self->{'results'}->{$address}->{'result'}}[$number];
        push @{$self->{'command_addresses'}}, $address;
        push @{$self->{'command_results'}}, $result;
        $self->{'number_of_results'}++;
      }
      elsif ($type eq "byte")
      {
        $result = ${$self->{'results'}->{$address}->{'result'}}[$number];
        push @{$self->{'command_addresses'}}, $address;
        push @{$self->{'command_results'}}, $result;
        $self->{'number_of_results'}++;
      }
      elsif ($type eq "word")
      {
        $result = ((${$self->{'results'}->{$address}->{'result'}}[$number*2] * 256) + ${$self->{'results'}->{$address}->{'result'}}[($number*2)+1] );
        push @{$self->{'command_addresses'}}, $address;
        push @{$self->{'command_results'}}, $result;
        $self->{'number_of_results'}++;
      }
      elsif ($type eq "dword")
      {
        $result = ((${$self->{'results'}->{$address}->{'result'}}[$number*4] * 16777216) + ${$self->{'results'}->{$address}->{'result'}}[($number*4)+1] * 65536);
        $result += ((${$self->{'results'}->{$address}->{'result'}}[($number*4)+2] * 256) + ${$self->{'results'}->{$address}->{'result'}}[($number*4)+3] );
        push @{$self->{'command_addresses'}}, $address;
        push @{$self->{'command_results'}}, $result;
        $self->{'number_of_results'}++;
      }
      elsif ($type eq "string")
      {
        $result = "";
        foreach (@{$self->{'results'}->{$address}->{'result'}})
        {
          if ($number > 0)
          {
            $number--;
          }
          else
          {
            if ($_ > 32 && $_ < 127)  # Ignore non-printable characters
            {
              $result .= chr($_);
            }
          }
        }
        push @{$self->{'command_addresses'}}, $address;
        push @{$self->{'command_results'}}, $result;
        $self->{'number_of_results'}++;
      }
    }
    else
    {
      $result = 0;
    }
  }
  
  return $result;
}


#*****************************************************************

=head2 ShowTroubleCodes

Display any trouble codes on the console:

	$obd->ShowTroubleCodes();
=cut

sub ShowTroubleCodes
{
	my ($self) = @_;

  my $malfunction_indicator_lamp_mask = 128;
  my $number_of_codes_mask = 127;

  if ($self->{'debug_level'} > 0) { print "ShowTroubleCodes\n"; }

  $self->Command("01 01");  # Get number of trouble codes

  my $number_of_codes = $self->GetResult("byte");
  my $malfunction_indicator_lamp_state = $number_of_codes & $malfunction_indicator_lamp_mask;
  $number_of_codes &= $number_of_codes_mask;
   
  if ($number_of_codes > 0)
  {
    $self->DisplayTroubleCodes();
  }
  else
  {
    print "No trouble codes found.\n";
  }
}


#*****************************************************************

# This function is called by ShowTroubleCodes. It displays the any
# trouble codes on the console.

sub DisplayTroubleCodes
{
	my ($self) = @_;

  my @codes = ("P0","P1","P2","P3","C0","C1","C2","C3","B0","B1","B2","B3","U0","U1","U2","U3");

  my $code_prefix_mask = 61440;
  my $code_mask = 4095;
  my $number_of_codes_mask = 127;
  my $result = 0;
  $self->{'trouble_codes'} = [];

  if ($self->{'debug_level'} > 0) { print "~DisplayTroubleCodes\n"; }

  $self->Command("03");  # Get trouble codes
  
  foreach my $address (sort keys %{$self->{'results'}})
  {
    if ($self->{'debug_level'} > 1)
    {
      print "Message Type: $self->{'results'}->{$address}->{'format'}\n";
      print "$address\n";
    }
    if ($self->{'results'}->{$address}->{'command'} == $self->{'last_command'})
    {
      my $index = 0;
      do
      {
        $result = ((${$self->{'results'}->{$address}->{'result'}}[$index] * 256) + ${$self->{'results'}->{$address}->{'result'}}[$index+1] );
        if ($result !=0)
        {
          push @{$self->{'command_addresses'}}, $address;
          push @{$self->{'command_results'}}, $result;
          $self->{'number_of_results'}++;
        }
        $index += 2;
      } while ($result != 0);
    }
  }

  if ($self->{'debug_level'} > 1)
  {
    print "Error code(s): @{$self->{'command_results'}}\n";
  }
  
  foreach my $code (@{$self->{'command_results'}})
  {
    my $code_prefix = ($code & $code_prefix_mask) >> 12;
    $code &= $code_mask;
    $code = sprintf("%03X", $code);
    my $decoded_code = "$codes[$code_prefix]$code";
    push @{$self->{'trouble_codes'}} , $decoded_code;
          
    if ($self->{'debug_level'} > 1)
    {
      print "Code prefix: $code_prefix, Code: $code, Decoded: $decoded_code\n";
    }
  }
}

#*****************************************************************

=head2 ClearTroubleCodes

Clear any Trouble Codes and sensor data:

	$obd->ClearTroubleCodes();

Note that clearing the sensor data will cause the vehicle to run on
default values until it has recalibrated itself. This may affect
performance and fuel economy.

The ISO specification also insists that any user interface which
invokes this function should display an "are you sure?" message
to the user before calling it.

=cut

sub ClearTroubleCodes
{
	my ($self) = @_;

  if ($self->{'debug_level'} > 0) { print "~ClearTroubleCodes\n"; }
  $self->Command("04");  # Clear Trouble Codes and sensor data

  my $result = $self->GetResult("byte");
  return $result;  # Returns 0 if codes have been cleared or error code.
}


#*****************************************************************

=head2 GetVIN

Returns a string containing the Vehicle Identification Number (VIN)

	my $vin = $obd->GetVIN();

This value can also be obtained using the Show and Read commands:

	$obd->Show("Vehicle Identification Number");
=cut

sub GetVIN
{
	my ($self) = @_;
  
  if ($self->{'debug_level'} > 0) { print "~GetVIN:\n"; }

  $self->Command("09 02");  # Get VIN

  my $vin = $self->GetResult("string", 1);

  return $vin;
}


#*****************************************************************

=head2 GetELM

Returns a string containing the type of the ELM module, e.g.:
"ELM327 v1.3a"

	my $elm = $obd->GetELM();

This value can also be obtained using the Show and Read commands:

	$obd->Read("ELM identity");
=cut

sub GetELM
{
	my ($self) = @_;

  if ($self->{'debug_level'} > 0) { print "~GetELM:\n"; }

  $self->Command("AT I");  # Read Identity
  return ${$self->{'response'}}[0];
}


#*****************************************************************

=head2 GetVoltage

Returns a string containing the Voltage measured by the ELM module 
(e.g. "13.9V"):

	my $Voltage = $obd->GetVoltage();

The number and unit can be obtained separately by calling:

	my $response = $obd->read("Input Voltage");
=cut

sub GetVoltage
{
	my ($self) = @_;

  if ($self->{'debug_level'} > 0) { print "~GetVoltage:\n"; }
  
  $self->Command("AT RV");  # Read Voltage
  return ${$self->{'response'}}[0];
}


#*****************************************************************

=head2 CalibrateVoltage

Changes the calibration value used by the ELM module. The value
$Voltage is a string containing a fixed point value of the form:
xx.xx, e.g "11.99", "12.01" etc.

	$obd->CalibrateVoltage($Voltage);
=cut

sub CalibrateVoltage
{
	my ($self, $Voltage) = @_;

  if ($self->{'debug_level'} > 0) { print "~CalibrateVoltage: $Voltage\n"; }
  
  $self->Command("AT CV $Voltage");  # Calibrate Voltage
  return ${$self->{'response'}}[0];
}


#*****************************************************************

=head2 ResetVoltage

Resets the ELM module's Voltage calibration to the factory setting:

	$obd->ResetVoltage();
=cut

sub ResetVoltage
{
	my ($self) = @_;

  if ($self->{'debug_level'} > 0) { print "~ResetVoltage:\n"; }

  $self->Command("AT CV 0000");  # Reset Voltage to factory setting
  return ${$self->{'response'}}[0];
}


#*****************************************************************

# Returns ignition state (ON or OFF)

sub GetIgnition
{
	my ($self) = @_;

  if ($self->{'debug_level'} > 0) { print "~GetIgnition:\n"; }
  
  $self->Command("AT IGN");  # Read ignition state (ON or OFF)
  return ${$self->{'response'}}[0];
}


#*****************************************************************

=head2 GetStoredDataByte

Returns the value previously stored in the ELM module's non-volatile
storage area:

	my $byte_value = $obd->WriteStoredDataByte();

This value can also be read using:

	$obd->Read("Stored data byte");
=cut

sub GetStoredDataByte
{
	my ($self) = @_;

  if ($self->{'debug_level'} > 0) { print "~GetStoredDataByte:\n"; }
  
  $self->Command("AT RD");  # Read the stored data
  return ${$self->{'response'}}[0];
}


#*****************************************************************

=head2 WriteStoredDataByte

Writes $byte_value to the ELM module's non-volatile storage area.

	$obd->WriteStoredDataByte($byte_value);
=cut

sub WriteStoredDataByte
{
	my ($self, $value) = @_;

  if ($self->{'debug_level'} > 0) { print "~WriteStoredDataByte: $value\n"; }
  
  $self->Command("AT SD $value");  # Save Data byte value
  return ${$self->{'response'}}[0];
}


#*****************************************************************

=head1 AUTHOR

Alister Perrott, C<< <aperrott at cpan.org> >>

=head1 BUGS

Please report any bugs or feature requests to C<bug-device-elm327 at rt.cpan.org>, or through
the web interface at L<http://rt.cpan.org/NoAuth/ReportBug.html?Queue=Device-ELM327>. 
I will be notified, and then you'll automatically be notified of progress on your bug as I make changes.

Please also include a debug trace showing the error. This can be done by setting $debug_level in the constructor to 1 and piping the output to a file.
e.g. perl myOBD.pl&>trace.txt


=head1 SUPPORT

You can find documentation for this module with the perldoc command.

	perldoc Device::ELM327


You can also look for information at:

=over 5

=item * RT: CPAN's request tracker

L<http://rt.cpan.org/NoAuth/Bugs.html?Dist=Device-ELM327>

=item * AnnoCPAN: Annotated CPAN documentation

L<http://annocpan.org/dist/Device-ELM327>

=item * CPAN Ratings

L<http://cpanratings.perl.org/d/Device-ELM327>

=item * Search CPAN

L<http://search.cpan.org/dist/Device-ELM327/>

=item * My ELM327 page

L<http://gts-ltd.co.uk/ELM327.php>

=back


=head1 ACKNOWLEDGEMENTS

Many thanks to:
	The authors of Win32::SerialPort and Device::SerialPort.
	Larry Wall and all the other people who have worked on Perl.
	ELM Electronics for creating the ELM327 module.
	Everyone involved with CPAN.

=head1 LICENSE AND COPYRIGHT

Copyright 2012 Alister Perrott.

This program is free software; you can redistribute it and/or modify it
under the terms of either: the GNU General Public License as published
by the Free Software Foundation; or the Artistic License.

See http://dev.perl.org/licenses/ for more information.


=cut

#********************************************************************
1; # End of Device::ELM327 - Return success to require/use statement
#********************************************************************

