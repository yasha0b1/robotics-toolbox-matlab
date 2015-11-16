classdef Torobot < Machine
    properties
        serPort;
        nservos;
        defaultSpeed;
    end
    
    properties (Constant)
        %%% define some important memory locations
        rad= pi/180;
        deg= 180/pi;
    end
    methods
        function torb = Torobot(varargin)
            %Torobot.Torobot Create Torobot interface object
            %
            % torb = Torobot(OPTIONS) is an object that represents a connection to a chain
            % of Torobot servos connected via an Torobot controller and serial link to the
            % host computer.
            %
            % Options::
            %  'port',P      Name of the serial port device, eg. /dev/tty.USB0
            %  'baud',B      Set baud rate (default 9600)
            %  'debug',D     Debug level, show communications packets (default 0)
            %  'nservos',N   Number of servos in the chain
            opt.port = '';
            opt.debug = false;
            opt.nservos = [];
            opt.baud = 9600;
            opt = tb_optparse(opt, varargin);
            torb.defaultSpeed = 1000;
            torb.serPort = opt.port;
            torb.debug = opt.debug;
            torb.nservos = opt.nservos;
            torb.connect(opt);

        end
        function connect(torb, opt)
            %Torobot.connect  Connect to the physical robot controller
            %
            % torb.connect() establish a serial connection to the physical robot
            % controller.
            %
            % See also Torobot.disconnect.
            % clean up any previous instances of the port, can happen...
            for tty = instrfind('port', opt.port)
                if ~isempty(tty)
                    disp(['serPort ' tty.port 'is in use.   Closing it.'])
                    fclose(tty);
                    delete(tty);
                end
            end
            if opt.verbose
                disp('Establishing connection to Torobot chain...');
            end
            %Communication protocol: serial communication (TTL level).
            %baud rate 9600, no check bit, 8 data bits, 1 stop bit
            torb.serPort = serial(torb.serPort,'BaudRate',opt.baud,...
                'DataBits',8,...
                'Parity','none',...
                'StopBits',1);
            set(torb.serPort,'InputBufferSize',1000)
            set(torb.serPort, 'Timeout', 1)
            set(torb.serPort, 'Tag', 'Torobot16')
            if opt.verbose
                disp('Opening connection to Torobot chain...');
            end
            pause(0.5);
            try
                fopen(torb.serPort);
            catch me
                disp('open failed');
                me.message
                return
            end
            torb.flush();
        end
        function disconnect(torb)
            %Torobot.disconnect  Disconnect from the physical robot controller
            %
            % torb.disconnect() closes the serial connection.
            %
            % See also Torobot.connect.
            tty = instrfind('port', torb.serPort.port);
            fclose(tty);
            delete(tty);
        end
        function setpos(torb, varargin)
            %Torobot.setpos Set position
            %
            % torb.SETPOS(ID, POS) vector POS (1xN) sets the position
            % corresponding to servos id indexed by vector ID (1xN).
            % torb.SETPOS(ID, POS, SPEED) as above but also sets the speed.
            
            % Notes::
            % -POS sets the position in degrees (-90,90),
            % -ID is in the range of 1-24(16 bit torobot controller).
            % -SPEED refers to the time of execution and represents the
            % speed of (1xN) servos. Regardless of the number of servos,
            % there is only one time,in the range 100-9999
            % See also Torobot.a2pwm.
            cmd='';
            id = varargin{1}
            pos = varargin{2}
            if isempty(torb.nservos)
                error('RTB:Torobot:notspec',...
                    'Number of servos not specified');
            end
            if length(pos) ~= length(pos)
                error('RTB:Torobot:badarg',...
                    'Length of POS vector must match number of ID');
            end
            for j=1:length(id)
                format=strcat('#%dP',num2str(torb.a2pwm(pos(j)*Torobot.deg)));
                num2str(id(j),format);
                cmd=strcat(cmd,num2str(id(j),format));
            end
            if nargin == 4
                speed = num2str(varargin{3},'%d');
            else
                speed = num2str(torb.defaultSpeed,'%d');
            end
            cmd=strcat(cmd,strcat('T',speed));
            torb.command(cmd);
            pause(str2num(speed)/1000);
        end
        function out = command(torb,  data)
            %Torobot.command Execute command on servo
            %
            % R = torb.COMMAND(INSTRUC) executes the instruction INSTRUC
            %
            % The optional output argument R is a structure holding the return status.
            %
            % Notes::
            % - If 'debug' was enabled in the constructor then the char values are echoed
            %   to the screen as well as being sent to the Torobot.
            % - If an output argument is requested the serial channel is flushed first.
            %
            if nargout > 0
                torb.flush();
            end
            out=sprintf('%s\r\n', data);
            if torb.debug > 0
                fprintf('send:    ');
                fprintf(out);
            end
            fwrite(torb.serPort, out);
            if nargout > 0
                out = torb.receive();
            end
        end
        function s = char(torb)
            %Torobot.char  Convert Torobot status to string
            %
            % C = Torobot.char() is a string that succinctly describes the status
            % of the Torobot controller link.
            
            % show serport Status, number of servos
            s = sprintf('Torobot chain on serPort %s (%s)', ...
                torb.serPort.port, get(torb.serPort, 'Status'));
            if torb.nservos
                s = strvcat(s, sprintf(' %d servos in chain', torb.nservos));
            end
        end
        function p = getpos(torb)
            %Torobot.getpos Get position
            % since the controller isn't designed for feedback enabled servos 
            % getting servo position is't possible with a torobot controller
            % TODO:
            % - maintain servo postion in memroy
            flushValue=torb.flush();
            p=char(flushValue.');
        end
        function setpath(torb, jt, t)
            %Arbotix.setpath Load a path into Arbotix controller
            %
            % ARB.setpath(JT) stores the path JT (PxN) in the Arbotix controller
            % where P is the number of points on the path and N is the number of
            % robot joints.  Allows for smooth multi-axis motion.

           
            
            if nargin < 3
                t = torb.defaultSpeed;    % milliseconds between poses
            end

            % set the poses
            %  payload: <pose#> q1 q2 .. qN
            steps=numrows(jt);
            torb.setpos(1:numcols(jt), jt(1,:),1000)
            if steps<2
                return
            end
            for i=2:steps
                torb.setpos(1:numcols(jt), jt(i,:),t)
            end

            
        end
        function s = receive(torb)
            %Torobot.receive Decode Torobot return packet
            %
            % R = torb.RECEIVE() reads and parses the return strings
            %
            % Notes::
            % - Some Torobot commands also return diagnostic text information.
            % - If 'debug' was enabled in the constructor then the hex values are echoed
            %
            % See also Torobot.command, Torobot.flush.
            N = torb.serPort.BytesAvailable();
            disp(N);
            if torb.debug > 0
                fprintf('receive: ');
            end
            cmdStop=[];
            cmdMsg=[];
            s='';
            stopState=false;
            state=0;
            while ( ~stopState && N ~= 0)
                c = fread(torb.serPort, 1, 'uint8');
                disp(c);
                disp(N);
                switch state
                    case 0
                        cmdStop=[cmdStop,c];
                        if(c==35)
                            state=1;
                        end
                        if(c==65)
                            stopState=true;
                            cmdMsg=[cmdMsg,c];
                        end
                    case 1
                        if(c==13 || c==10 )
                            cmdStop=[cmdStop,c];
                            stopState=strcmp(sprintf('%s%s',cmdStop),sprintf('%s%s',13,10));
                        else
                            cmdStop=[];
                            cmdMsg=[cmdMsg,c];
                        end              
                end
                N = torb.serPort.BytesAvailable();
            end
            s=char(cmdMsg);
        end
        function out = flush(robot)
            %Torobot.flush Flush the receive buffer
            %
            % torb.FLUSH() flushes the serial input buffer, data is discarded.
            %
            % S = torb.FLUSH() as above but returns a vector of all bytes flushed from
            % the channel.
            %
            % See also Torobot.receive, Torobot.parse.
            %Flush Buffer
            N = robot.serPort.BytesAvailable();
            data = [];
            % this returns a maximum of input buffer size
            while (N ~= 0)
                data = [data, fread(robot.serPort, N)];
                pause(0.1); % seem to need this
                N = robot.serPort.BytesAvailable();
            end
            if nargout > 0
                out=char(data.');
            end
        end
    end
    
    methods(Static)
        function a = pwm2a(pwm)
            %Torobot.pwm2a Convert pwm to angle
            % 11.11us=(1500us-pwm)/(90°-deg)
            a = floor((pwm-500.1)/11.11);
        end
        function pwm = a2pwm(a)
            %Torobot.a2pwm Convert angle to pwm
            % 11.11us=(1500us-pwm)/(90°-deg)
            pwm = floor(500.1 + 11.11*(a));
        end
    end
end