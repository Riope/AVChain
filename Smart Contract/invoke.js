const {Wallets, Gateway} = require('fabric-network')
const FabricCAServices = require('fabric-ca-client')
const {buildCAClient, registerAndEnrollUser, enrollAdmin} = require('../test-application/javascript/CAUtil.js');
const {buildCCPOrg1, buildWallet} = require('../test-application/javascript/AppUtil.js');
const fs = require('fs')
const path = require('path')
const channelName = 'mychannel';
const chaincodeName = 'basic';
const mspOrg1 = 'Org1MSP';
const walletPath = path.join(__dirname, 'wallet');
// const orgUserId = 'appUser';

// function prettyJSONString(inputString) {
// 	return JSON.stringify(JSON.parse(inputString), null, 2);
// }

async function main(){
    const args = process.argv.slice(3);
    try {
        if (args.length == 0) {
            throw new Error('Specify a command')
        }
    } catch(err) {
        console.log(`Insert Invokation [Caught exception] : ${err}`)
    }

    // Org1 connection profile
    var ccp = buildCCPOrg1();
    
    // Org1 Ca
    const caClient = buildCAClient(FabricCAServices, ccp, 'ca.org1.example.com');

    // Create a wallet instance
    const wallet = await buildWallet(Wallets, walletPath);

    // Enroll the admin
    const Adminidentity = await wallet.get('admin');
    if (Adminidentity) {
        console.log('An identity for the admin user "admin" already exists in the wallet');
    } else {
        await enrollAdmin(caClient, wallet, mspOrg1);
    }

    // Register a user
    let orgUserId = process.argv[2]
    const identity = await wallet.get(orgUserId);
    if (identity) {
        console.log('An identity for the user' + orgUserId + 'already exists in the wallet');
    } else {
        if (orgUserId == "Alice") {
            await registerAndEnrollUser(caClient, wallet, mspOrg1, orgUserId, 'automaker', 'org1.department1');
        } else if (orgUserId == "Bob") {
            await registerAndEnrollUser(caClient, wallet, mspOrg1, orgUserId, 'sensormanufacturer', 'org1.department1');
        } else if (orgUserId == "Cathy") {
            await registerAndEnrollUser(caClient, wallet, mspOrg1, orgUserId, 'actuatorsupplier', 'org1.department1');
        }
    }

    var gateway, network, contract;
    try {
        // Connect to gateway
        gateway = new Gateway();
        await gateway.connect(ccp, {wallet, identity: orgUserId, discovery: {enabled:true, asLocalhost: true}})
        
        // Connect to channel
        network = await gateway.getNetwork(channelName)
        
        // Select the contract
        contract = network.getContract(chaincodeName)
    } catch(err) {
        console.log(`Caught exception: ${err}`)
        return
    }

    switch (args[0]) {
        case 'PushData':
            try {
                let ch = ","
                for (let i=3; i<args.length; i+=2) {
                    if (i==3) {
                        args[i] = args[i].slice(0,1) + "\"" + args[i].slice(1,args[i].indexOf(":")) + "\"" + ":"
                    }
                    else {
                        args[i] = "\"" + args[i].slice(0,args[i].indexOf(":")) + "\"" + ":"
                        if (i+2 == args.length) {
                            ch = "}"
                        }
                        args[i+1] = "\"" + args[i+1].slice(0,args[i+1].indexOf(ch)) + "\"" + ch
                    }
                }
                const data = args.slice(3, args.length).join(" ")

                await contract.submitTransaction("PushData", args[1], args[2], data)
                console.log("Insert Invokation Committed.")
            } catch(err) {
                console.log(`Insert Invokation [Caught exception] : ${err}`)
            }
            break;

        case 'ReadCam0Data':
            try {
                if (args.length != 4) {
                    throw new Error('Incorrect number of arguments')
                }
                let result = await contract.evaluateTransaction("ReadCam0Data", args[1], args[2], args[3])
                console.log(`Successful ReadCam0Data Query: ${result}`)
            } catch(err) {
                console.log(`Unsuccessful ReadCam0Data Query [Caught exception] : ${err}`)
            }
            break;
        
        case 'ReadCam1Data':
            try {
                if (args.length != 4) {
                    throw new Error('Incorrect number of arguments')
                }
                let result = await contract.evaluateTransaction("ReadCam1Data", args[1], args[2], args[3])
                console.log(`Successful ReadCam1Data Query: ${result}`)
            } catch(err) {
                console.log(`Unsuccessful ReadCam1Data Query [Caught exception] : ${err}`)
            }
            break;

        case 'ReadLIDARData':
            try {
                if (args.length != 4) {
                    throw new Error('Incorrect number of arguments')
                }
                let result = await contract.evaluateTransaction("ReadLIDARData", args[1], args[2], args[3])
                console.log(`Successful ReadLIDARData Query: ${result}`)
            } catch(err) {
                console.log(`Unsuccessful ReadLIDARData Query [Caught exception] : ${err}`)
            }
            break;
        
        case 'ReadSpeedData':
            try {
                if (args.length != 4) {
                    throw new Error('Incorrect number of arguments')
                }
                let result = await contract.evaluateTransaction("ReadSpeedData", args[1], args[2], args[3])
                console.log(`Successful ReadSpeedData Query: ${result}`)
            } catch(err) {
                console.log(`Unsuccessful ReadSpeedData Query [Caught exception] : ${err}`)
            }
            break;
        
        case 'ReadAcclnData':
            try {
                if (args.length != 4) {
                    throw new Error('Incorrect number of arguments')
                }
                let result = await contract.evaluateTransaction("ReadAcclnData", args[1], args[2], args[3])
                console.log(`Successful ReadAcclnData Query: ${result}`)
            } catch(err) {
                console.log(`Unsuccessful ReadAcclnData Query [Caught exception] : ${err}`)
            }
            break;

        case 'ReadGyroData':
            try {
                if (args.length != 4) {
                    throw new Error('Incorrect number of arguments')
                }
                let result = await contract.evaluateTransaction("ReadGyroData", args[1], args[2], args[3])
                console.log(`Successful ReadGyroData Query: ${result}`)
            } catch(err) {
                console.log(`Unsuccessful ReadGyroData Query [Caught exception] : ${err}`)
            }
            break;

        case 'ReadCompassData':
            try {
                if (args.length != 4) {
                    throw new Error('Incorrect number of arguments')
                }
                let result = await contract.evaluateTransaction("ReadCompassData", args[1], args[2], args[3])
                console.log(`Successful ReadCompassData Query: ${result}`)
            } catch(err) {
                console.log(`Unsuccessful ReadCompassData Query [Caught exception] : ${err}`)
            }
            break;
        
        case 'ReadThrottleData':
            try {
                if (args.length != 4) {
                    throw new Error('Incorrect number of arguments')
                }
                let result = await contract.evaluateTransaction("ReadThrottleData", args[1], args[2], args[3])
                console.log(`Successful ReadThrottleData Query: ${result}`)
            } catch(err) {
                console.log(`Unsuccessful ReadThrottleData Query [Caught exception] : ${err}`)
            }
            break;

        case 'ReadSteerData':
            try {
                if (args.length != 4) {
                    throw new Error('Incorrect number of arguments')
                }
                let result = await contract.evaluateTransaction("ReadSteerData", args[1], args[2], args[3])
                console.log(`Successful ReadSteerData Query: ${result}`)
            } catch(err) {
                console.log(`Unsuccessful ReadSteerData Query [Caught exception] : ${err}`)
            }
            break;

        case 'ReadBrakeData':
            try {
                if (args.length != 4) {
                    throw new Error('Incorrect number of arguments')
                }
                let result = await contract.evaluateTransaction("ReadBrakeData", args[1], args[2], args[3])
                console.log(`Successful ReadBrakeData Query: ${result}`)
            } catch(err) {
                console.log(`Unsuccessful ReadBrakeData Query [Caught exception] : ${err}`)
            }
            break;

        case 'ReadGearData':
            try {
                if (args.length != 4) {
                    throw new Error('Incorrect number of arguments')
                }
                let result = await contract.evaluateTransaction("ReadGearData", args[1], args[2], args[3])
                console.log(`Successful ReadGearData Query: ${result}`)
            } catch(err) {
                console.log(`Unsuccessful ReadGearData Query [Caught exception] : ${err}`)
            }
            break;

        case 'ReadHandBrakeData':
            try {
                if (args.length != 4) {
                    throw new Error('Incorrect number of arguments')
                }
                let result = await contract.evaluateTransaction("ReadHandBrakeData", args[1], args[2], args[3])
                console.log(`Successful ReadHandBrakeData Query: ${result}`)
            } catch(err) {
                console.log(`Unsuccessful ReadHandBrakeData Query [Caught exception] : ${err}`)
            }
            break;

        case 'ReadVehicleFrames':
            try {
                if (args.length != 2) {
                    throw new Error('Incorrect number of arguments')
                }
                let result = await contract.evaluateTransaction("ReadVehicleFrames", args[1])
                console.log(`Successful ReadVehicleFrames Query: ${result}`)
            } catch(err) {
                console.log(`Unsuccessful ReadVehicleFrames Query [Caught exception] : ${err}`)
            }
            break;
        
        default:
            console.log("Incorrect Command")
    }

    gateway.disconnect();
}

main();