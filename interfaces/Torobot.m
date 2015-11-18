classdef Torobot < Machine
    properties
        serPort;
        nservos;
        defaultSpeed;
        actionGroups;
        servoPos
    end
    
    properties (Constant)
        %%% define some important memory locations
        rad= pi/180;
        deg= 180/pi;
        REC_VERSION_OK = 1;
        REC_DOWN_ADDED = 2;
        REC_DOWN_OK =3;
        REC_READ_OK=4
        REC_CLEAR_CONTINUE=5;
        REC_CLEAR_OK=6;
        REC_AG_FINISH=7;
        REC_ERROR = -1;
        
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
            torb.actionGroups=0;
            torb.servoPos=NaN(1,torb.nservos);

            
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
            
            
            try
                fopen(torb.serPort);
            catch me
                disp('open failed');
                me.message
                return
            end
            pause(2);
            disp(torb.serPort.Status);
            torb.flush();
            torb.command('#Veri+201405201212');
            s=torb.receive();
            if s~=torb.REC_VERSION_OK
                error('Controller version check failed');
            else
                torb.command('#VERI');
            end
            
            
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
            % torb.SETPOS(ID, POS) sets the position (0-1023) of servo ID.
            % torb.SETPOS(ID, POS, SPEED) as above but also sets the speed.
            %
            % torb.SETPOS(POS) sets the position of servos 1-N to corresponding elements
            % of the vector POS (1xN).
            % torb.SETPOS(POS, SPEED) as above but also sets the velocity SPEED.
            %
            % Notes::
            % - ID is in the range 1 to N
            % - N is defined at construction time by the 'nservos' option.
            % - SPEED is a measured in time(milliseconds) of execution and
            %   represents the rate of movement , 
            %   Regardless of the number of servos,
            %   there is only one time,in the range 100-9999. 
            cmd='';
            if length(varargin{1}) > 1
                % vector mode
                pos = varargin{1};
                
                if isempty(torb.nservos)
                    error('RTB:Torobot:notspec', 'Number of servos not specified');
                end
                if length(pos) ~= torb.nservos
                    error('RTB:Torobot:badarg', 'Length of POS vector must match number of servos');
                end
                if nargin == 3
                    speed = num2str(varargin{2},'%d');
                else
                    speed = num2str(torb.defaultSpeed,'%d');
                end
                id=1:torb.nservos;
            else
                % single joint mode
                id = varargin{1}; 
                pos = varargin{2};
                if nargin == 4
                    speed = num2str(varargin{3},'%d');
                else
                    speed = num2str(torb.defaultSpeed,'%d');
                end
            end
            for j=1:length(id)
                format=strcat('#%dP',num2str(torb.a2pwm(pos(j)*Torobot.deg)));
                num2str(j,format);
                cmd=strcat(cmd,num2str(j,format));
            end
            cmd=strcat(cmd,strcat('T',speed));
            torb.command(cmd);
            pause(str2num(speed)/1000);
            torb.servoPos(id)=pos;
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
            % getting servo position is't possible with a torobot
            % controller.
            % the servo pose is maintained in memopry.
            p=torb.servoPos;
        end
        function clearAG(torb)
            %Torobot.clearAG Clears all action groups in controller nand memory
            %
            % See also Torobot.executeAG, Torobot.readAG, Torobot.setAG
            torb.flush();
            torb.command('#Clear');
            torb.receive();
            s=torb.receive();
            while s~=torb.REC_CLEAR_OK
                if s~=torb.REC_CLEAR_CONTINUE
                    error('Action clearing failed');
                end
                s=torb.receive();
            end
            torb.actionGroups=-1;
        end
        function stop(torb)
           torb.command('#STOP');
        end
        function executeAG(torb, ag,cycles,timeout)
            %Torobot.executeAG execute action groups
            % Execute multiple action groups
            % -ag (1xN) where n is a sequence of action group
            % -cycles(optional) the number of iterations to execute 
            %  action groups
            % -timeout(optional) time to wait for action group sequence to
            %  complete
            % See also Torobot.clear, Torobot.readAG, Torobot.setAG
            if nargin < 4                
                timeout = 60*1000;    % default timeout
            end
            if nargin < 3
                cycles = 1;    % default to one cycle 
            end
            torb.flush();
            torb.readAG();
            str='';
            for i=1:length(ag)
                if ag(i)>torb.actionGroups || ag(1)<=0
                    error('action group %d does not exist', ag(i));
                else
                    cmd=strcat('#',strcat(num2str(ag(i)),'G'));
                    str=strcat(str,cmd);
                end
            end
            torb.flush();
            torb.command(strcat(str,strcat('C',num2str(cycles))));
            torb.servoPos=NaN(1,torb.nservos);
            s=torb.receive(timeout); %set timeout to 1 minute
            if s~=torb.REC_AG_FINISH
                torb.flush();
                for i=1:cycles
                    torb.stop();
                    pause(.1)
                end
                error('Action group execution failed');
            end
        end
        function setAG(torb, jt, t)
            %Torobot.setAG sets an action group in the torobot controller
            % torb.setAG(jt) sets an action group JT (PxN) in the Torobot controller
            % where P is the number of points on the path and N is the number of
            % robot joints.  
            % torb.setAG(jt,t) same as above but assignes milliseconds 
            % between poses
            %See also Torobot.clearAG, Torobot.readAG, Torobot.executeAG
            if nargin < 3
                t = torb.defaultSpeed;    % milliseconds between poses
            end
            steps=numrows(jt);
            torb.flush();
            torb.command('#Down');
            s=torb.receive();
            if s~=torb.REC_DOWN_ADDED
                torb.command('#Stop');
                error('Action group download failed');
            end
            for i=1:steps
                torb.setpos(jt(i,:),t);
                s=torb.receive();
                if s~=torb.REC_DOWN_ADDED
                    torb.command('#Stop');
                    error('Action group download failed');
                end
            end
            torb.command('#Stop');
            s=torb.receive();
            if s~=torb.REC_DOWN_OK
                error('Action group download failed');
            end
            
        end
        function readAG(torb)
            %Torobot.readAG reads access groups numbers stored in
            % controller nand memory
            %
            % See also Torobot.executeAG, Torobot.clearAG, Torobot.setAG
            torb.flush();
            torb.command('#Read')
            s=torb.receive();
            if s~=torb.REC_READ_OK
                error('Read action groups failed');
            end
        end
        function setpath(torb, jt, t)
            %Torobot.setpath execute a path on the torobot controller
            %
            % torb.setpath(JT) executes path JT (PxN) in the Torobot controller
            % where P is the number of points on the path and N is the number of
            % robot joints.  Allows for smooth multi-axis motion.
            if nargin < 3                 % set default speed if non provided
                t = torb.defaultSpeed;    % milliseconds between poses
            end
            steps=numrows(jt);            % number of poses
            for i=1:steps
                torb.setpos(jt(i,:),t);
            end
        end
        function s = receive(torb,timeout)
            %Torobot.receive Decode Torobot return packet
            %
            % R = torb.RECEIVE() reads and parses the return strings
            % -timeout in milliseconds
            % See also Torobot.command, Torobot.flush. 
            checkTime=0.1;
            if nargin < 2                
                timeout = 500;    % default timeout
            end
            timeout=timeout/1000;
            if torb.debug > 0
                fprintf('receive: ');
            end
            cmdStop=[];
            cmdMsg=[];
            s='';
            c='';
            stopState=false;
            state=0;
            while ( ~stopState && (timeout > 0))
                if torb.serPort.BytesAvailable()>0
                    c = fread(torb.serPort, 1, 'uint8');
                end
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
                if torb.serPort.BytesAvailable()==0
                    pause(checkTime);
                    timeout = timeout - checkTime;
                end
            end
            str=char(cmdMsg);
            if torb.debug > 0
                fprintf('%s\n', str);
            end
            s=torb.retDecode(str);
            % set number of action groups returned from Torobot.setAG
            % function
            if s==torb.REC_DOWN_OK
                tok = regexp(str,'^Down\+OK\+([0-9]+)' , 'tokens');
                torb.actionGroups=str2double(tok{:});
            end
            % set number of action groups returned from Torobot.readAG
            % function
            if s==torb.REC_READ_OK
                tok = regexp(str,'^R\+OK\+([0-9]+)' , 'tokens');
                torb.actionGroups=str2double(tok{:});
            end
            
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
                data = [data; fread(robot.serPort, N)];
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
        function ret = retDecode(str)
            %Torobot.retDecode decodes return string from controller
            ret=Torobot.REC_ERROR;
            if(regexp(str,'^VERI[a-zA-Z0-9+]+OK')>0)
                ret=Torobot.REC_VERSION_OK;
            end
            if(strcmp(str,'A'))
                ret=Torobot.REC_DOWN_ADDED;
            end
            if(regexp(str,'^Down[a-zA-Z0-9+]+OK')>0)
                ret=Torobot.REC_DOWN_OK;
            end
            if(regexp(str,'^([0-9]+)GC'))
                ret=Torobot.REC_READ;
            end
            if(regexp(str,'C\+([0-9]+)\+OK'))
                ret=Torobot.REC_CLEAR_CONTINUE;
            end
            if(strcmp(str,'CLEAR+OK'))
                ret=Torobot.REC_CLEAR_OK;
            end
            if(strcmp(str,'AGF'))
                ret=Torobot.REC_AG_FINISH;
            end
            if(regexp(str,'R\+OK\+[0-9]+'))
                ret=Torobot.REC_READ_OK;
            end
            
        end
        function pwm = a2pwm(a)
            %Torobot.a2pwm Convert angle to pwm
            % 11.11us=(1500us-pwm)/(90°-deg)
            pwm = floor(500.1 + 11.11*(a));
        end
    end
end