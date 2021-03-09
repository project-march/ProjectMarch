# Setup libcap-dev
apt install -y libcap-dev &&

# Remove ethercat_grant folder if it already exists
rm -rf ethercat_grant/ &&

# Clone ethcat_grant package
git clone https://github.com/shadow-robot/ethercat_grant.git &&

# Build ethercat_grant package
cd ethercat_grant &&
mkdir build &&
cd build &&
cmake .. &&
cmake --build . &&

# Copy ethercat_Grant executable and give it the right permissions
cp devel/lib/ethercat_grant/ethercat_grant /usr/local/bin/ &&
chmod +s /usr/local/bin/ethercat_grant &&

# Clean up ethercat_grant folder
rm -rf ethercat_grant/ &&

echo '[MARCH] Installation of ethercat_grant succesfull!'