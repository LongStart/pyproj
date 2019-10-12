def shell_source(script):
    """Sometime you want to emulate the action of "source" in bash,
    settings some environment variables. Here is a way to do it."""
    import subprocess, os
    pipe = subprocess.Popen(". %s; env" % script, stdout=subprocess.PIPE, shell=True, executable='/bin/bash')
    output = pipe.communicate()[0].decode("utf-8")
    env = dict((line.split("=", 1) for line in output.splitlines()))
    os.environ.update(env)