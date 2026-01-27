

// Function to send a command
void send_command(int fd, char command)
{
    // write(fd, &command, 1);
}

void processGPOpacket(unsigned char data)
{
    for (int i = 7; i >= 4; i--)
    {
        int bit = (data >> i) & 1;
        if (bit != 0)
        {
            // printf("bit = %d\n", i);
        }
        // printf("Bit %d: %d (0x%x)\n", i, bit, bit << i);
        // if (i == 4)
        // {
        // printf("%c - bit: %d\n", bit ? 'a' : 'A', bit);
        // send_command(fdHW210, bit ? 'a' : 'A');
        // usleep(500);
        // }
        // else if (i == 6)
        // {
        // printf("%c - bit: %d\n", bit ? 'b' : 'B', bit);
        // send_command(fdHW210, bit ? 'b' : 'B');
        // usleep(500);
        // }
    }
}


