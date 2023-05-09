import paramiko
import re
import json

class ShellHandler:
	def __init__(self, host, user, path):
		self.ssh = paramiko.SSHClient()
		self.ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
		self.ssh.connect(host, username=user, key_filename=path, port=22)

		channel = self.ssh.invoke_shell()
		self.stdin = channel.makefile('wb')
		self.stdout = channel.makefile('r')

	def __del__(self):
		self.ssh.close()

	def execute(self, cmd):
		cmd = cmd.strip('\n')
		self.stdin.write(cmd + '\n')
		finish = 'end of stdOUT buffer. finished with exit status'
		echo_cmd = 'echo {} $?'.format(finish)
		self.stdin.write(echo_cmd + '\n')
		shin = self.stdin
		self.stdin.flush()

		shout = []
		sherr = []
		exit_status = 0
		for line in self.stdout:
			if str(line).startswith(cmd) or str(line).startswith(echo_cmd):
				# up for now filled with shell junk from stdin
				shout = []
			elif str(line).startswith(finish):
				# our finish command ends with the exit status
				exit_status = int(str(line).rsplit(maxsplit=1)[1])
				if exit_status:
					sherr = shout
					shout = []
				break
			else:
				# get rid of 'coloring and formatting' special characters
				shout.append(re.compile(r'(\x9B|\x1B\[)[0-?]*[ -/]*[@-~]').sub('', line).
								replace('\b', '').replace('\r', '').replace('\n',''))

		# first and last lines of shout/sherr contain a prompt
		if shout and echo_cmd in shout[-1]:
			shout.pop()
		if shout and cmd in shout[0]:
			shout.pop(0)
		if sherr and echo_cmd in sherr[-1]:
			sherr.pop()
		if sherr and cmd in sherr[0]:
			sherr.pop(0)

		return shin, shout, sherr

	def exec_cmd(self, cmd, return_cid=False):
		try:
			_, std_out, _ = self.execute(cmd)
			output = {"std_out": std_out}

			if return_cid:
				temp_str = std_out[-2]
				temp_str = temp_str[temp_str.index('[')+1:temp_str.index(']')]
				output['cids'] = temp_str.split(",")

			print(json.dumps(output))
			
		except Exception as error_message:
			print("Couldn't run command")
			print(error_message)
