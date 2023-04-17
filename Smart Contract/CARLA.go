package main

import (
	"encoding/json"
	"fmt"
	"log"
	"strconv"
	// "bytes"
	"crypto/md5"
    "encoding/hex"
	"github.com/hyperledger/fabric-contract-api-go/contractapi"
	"github.com/hyperledger/fabric-chaincode-go/pkg/cid"
	// shell "github.com/ipfs/go-ipfs-api"
)

type SmartContract struct {
	contractapi.Contract
}

type SensorData struct {
	FrameNo int `json:"FrameNo"`

	RGBCam0 string `json:"RGBCam0"`
	RGBCam1 string `json:"RGBCam1"`
	LIDAR string `json:"LIDAR"`
	Speed string `json:"Speed"`
	Accln string `json:"Accln`
	Gyro string `json:"Gyro`
	Compass string `json:"Compass`

	Throttle string `json:"Throttle"`
	Steering string `json:"Steering"`
	Braking string `json:"Braking"`
	Gear string `json:"Gear"`
	HandBrake string `json:"HandBrake"`
}

func GetHash(text string) string {
    hasher := md5.New()
    hasher.Write([]byte(text))
    return hex.EncodeToString(hasher.Sum(nil))
}

func Hash(a int, b float64) string {
	temp := GetHash(strconv.Itoa(a))
	temp = GetHash(temp + GetHash(strconv.FormatFloat(b, 'E', -1, 32)))
	return temp
}

func VehicleExists(ctx contractapi.TransactionContextInterface, v_id int) (bool, error) {
	assetJSON, err := ctx.GetStub().GetState(strconv.Itoa(v_id))

	if err != nil {
		return false, err
	}
	if assetJSON != nil {
		return true, nil
	}
	return false, nil
}

func ReadData(ctx contractapi.TransactionContextInterface, v_id int, timestamp float64, data_source string) (string, error) {
	id := Hash(v_id, timestamp)
	assetJSON, err := ctx.GetStub().GetState(id)

	if err != nil {
		return "", fmt.Errorf("failed to read from world state: %v", err)
	}
	if assetJSON == nil {
		status, err := VehicleExists(ctx, v_id)
		if err!=nil {
			return "", err
		} else if status {
			return "", fmt.Errorf("the time %s was not monitored", strconv.FormatFloat(timestamp, 'E', -1, 32))
		} else {
			return "", fmt.Errorf("the vehicle id %s does not exist", strconv.Itoa(v_id))
		}
	}

	// Convert That JSON to object of type SensorData
	var asset SensorData
	err = json.Unmarshal(assetJSON, &asset)
	if err != nil {
		return "", err
	}

	// sh := shell.NewShell("localhost:5001")
	// data, err := sh.Cat(asset.RGBCam0)
	// if err != nil {
	// 	return "", fmt.Errorf("failed to read from ipfs: %v", err)
	// }
	// buf := new(bytes.Buffer)
    // buf.ReadFrom(data)
    // newStr := buf.String()
	
	if data_source == "CAM0" {
		return asset.RGBCam0, nil
	} else if data_source == "CAM1" {
		return asset.RGBCam1, nil
	} else if data_source == "LIDAR" {
		return asset.LIDAR, nil
	} else if data_source == "Speed" {
		return asset.Speed, nil
	} else if data_source == "Accln" {
		return asset.Accln, nil
	} else if data_source == "Gyro" {
		return asset.Gyro, nil
	} else if data_source == "Compass" {
		return asset.Compass, nil
	} else if data_source == "Throttle" {
		return asset.Throttle, nil
	} else if data_source == "Steering" {
		return asset.Steering, nil
	} else if data_source == "Braking" {
		return asset.Braking, nil
	} else if data_source == "Gear" {
		return asset.Gear, nil
	} else if data_source == "HandBrake" {
		return asset.HandBrake, nil
	}
	return "", fmt.Errorf("incorrect data source")
}

func (s *SmartContract) ReadFrameData(ctx contractapi.TransactionContextInterface, v_id int, start_time float64, end_time float64, data_source string) ([]string, error) {
	timestamps, err := s.ReadVehicleFrames(ctx, v_id)
	cids := []string{}
	if err != nil {
		return cids, fmt.Errorf("failed to read timestamps: %v", err)
	}

	var cid string
	for _, timestamp := range timestamps {
		if timestamp > end_time {
			break
		}
		if timestamp >= start_time {
			cid, err = ReadData(ctx, v_id, timestamp, data_source)
			if err != nil {
				return cids, fmt.Errorf("failed to read data for time: %v", err)
			}
			cids = append(cids, cid)
		}
	}
	
	return cids, nil
}

func (s *SmartContract) PushData(ctx contractapi.TransactionContextInterface, v_id int, timestamp float64, data string) error {
	var asset SensorData
	err := json.Unmarshal([]byte(data), &asset)
	if err != nil {
		return err
	}

	var timestamps []float64
	status, err := VehicleExists(ctx, v_id)
	if err != nil {
		return err
	}
	if status {
		assetJSON, err := ctx.GetStub().GetState(strconv.Itoa(v_id))
		err = json.Unmarshal(assetJSON, &timestamps)
		if err != nil {
			return err
		}
	}
	
	timestamps = append(timestamps, timestamp)
	assetJSON, err := json.Marshal(timestamps)
	if err != nil {
		return err
	}
	err = ctx.GetStub().PutState(strconv.Itoa(v_id), assetJSON)
	if err != nil {
		return err
	}

	assetJSON, err = json.Marshal(asset)
	if err != nil {
		return err
	}
	id := Hash(v_id, timestamp)
	return ctx.GetStub().PutState(id, assetJSON)
}

func (s *SmartContract) ReadVehicleFrames(ctx contractapi.TransactionContextInterface, v_id int) ([]float64, error) {
	assetJSON, err := ctx.GetStub().GetState(strconv.Itoa(v_id))
	timestamps := []float64{}

	if err != nil {
		return timestamps, fmt.Errorf("failed to read from world state: %v", err)
	}
	if assetJSON == nil {
		return timestamps, fmt.Errorf("the vehicle id %s does not exist", strconv.Itoa(v_id))
	}

	// Convert That JSON to object of type SensorData
	err = json.Unmarshal(assetJSON, &timestamps)
	if err != nil {
		return timestamps, err
	}

	return timestamps, nil
}

func (s *SmartContract) ReadCam0Data(ctx contractapi.TransactionContextInterface, v_id int, start_time float64, end_time float64) ([]string, error) {
	objval, ok, err := cid.GetAttributeValue(ctx.GetStub(), "role")
	cids := []string{}
	if err != nil {
		return cids, fmt.Errorf("error while retrieving attributes: %v", err)
	}
	if !ok {
		return cids, fmt.Errorf("client identity does not possess the attribute: %v", err)
	}

	if (objval == "automaker") || (objval == "sensormanufacturer") {
		cids, err = s.ReadFrameData(ctx, v_id, start_time, end_time, "CAM0")
		if err != nil {
			return cids, fmt.Errorf("failed due to: %v", err)
		}
		return cids, nil
	}

	return cids, fmt.Errorf("user does not have access to function")
}

func (s *SmartContract) ReadCam1Data(ctx contractapi.TransactionContextInterface, v_id int, start_time float64, end_time float64) ([]string, error) {
	objval, ok, err := cid.GetAttributeValue(ctx.GetStub(), "role")
	cids := []string{}
	if err != nil {
		return cids, fmt.Errorf("error while retrieving attributes: %v", err)
	}
	if !ok {
		return cids, fmt.Errorf("client identity does not possess the attribute: %v", err)
	}

	if (objval == "automaker") || (objval == "sensormanufacturer") {
		cids, err = s.ReadFrameData(ctx, v_id, start_time, end_time, "CAM1")
		if err != nil {
			return cids, fmt.Errorf("failed due to: %v", err)
		}
		return cids, nil
	}

	return cids, fmt.Errorf("user does not have access to function")
}

func (s *SmartContract) ReadLIDARData(ctx contractapi.TransactionContextInterface, v_id int, start_time float64, end_time float64) ([]string, error) {
	objval, ok, err := cid.GetAttributeValue(ctx.GetStub(), "role")
	cids := []string{}
	if err != nil {
		return cids, fmt.Errorf("error while retrieving attributes: %v", err)
	}
	if !ok {
		return cids, fmt.Errorf("client identity does not possess the attribute: %v", err)
	}

	if (objval == "automaker") || (objval == "sensormanufacturer") {
		cids, err = s.ReadFrameData(ctx, v_id, start_time, end_time, "LIDAR")
		if err != nil {
			return cids, fmt.Errorf("failed due to: %v", err)
		}
		return cids, nil
	}

	return cids, fmt.Errorf("user does not have access to function")
}

func (s *SmartContract) ReadSpeedData(ctx contractapi.TransactionContextInterface, v_id int, start_time float64, end_time float64) ([]string, error) {
	objval, ok, err := cid.GetAttributeValue(ctx.GetStub(), "role")
	cids := []string{}
	if err != nil {
		return cids, fmt.Errorf("error while retrieving attributes: %v", err)
	}
	if !ok {
		return cids, fmt.Errorf("client identity does not possess the attribute: %v", err)
	}

	if (objval == "automaker") || (objval == "sensormanufacturer") {
		cids, err = s.ReadFrameData(ctx, v_id, start_time, end_time, "Speed")
		if err != nil {
			return cids, fmt.Errorf("failed due to: %v", err)
		}
		return cids, nil
	}

	return cids, fmt.Errorf("user does not have access to function")
}

func (s *SmartContract) ReadAcclnData(ctx contractapi.TransactionContextInterface, v_id int, start_time float64, end_time float64) ([]string, error) {
	objval, ok, err := cid.GetAttributeValue(ctx.GetStub(), "role")
	cids := []string{}
	if err != nil {
		return cids, fmt.Errorf("error while retrieving attributes: %v", err)
	}
	if !ok {
		return cids, fmt.Errorf("client identity does not possess the attribute: %v", err)
	}

	if (objval == "automaker") || (objval == "sensormanufacturer") {
		cids, err = s.ReadFrameData(ctx, v_id, start_time, end_time, "Accln")
		if err != nil {
			return cids, fmt.Errorf("failed due to: %v", err)
		}
		return cids, nil
	}

	return cids, fmt.Errorf("user does not have access to function")
}

func (s *SmartContract) ReadGyroData(ctx contractapi.TransactionContextInterface, v_id int, start_time float64, end_time float64) ([]string, error) {
	objval, ok, err := cid.GetAttributeValue(ctx.GetStub(), "role")
	cids := []string{}
	if err != nil {
		return cids, fmt.Errorf("error while retrieving attributes: %v", err)
	}
	if !ok {
		return cids, fmt.Errorf("client identity does not possess the attribute: %v", err)
	}

	if (objval == "automaker") || (objval == "sensormanufacturer") {
		cids, err = s.ReadFrameData(ctx, v_id, start_time, end_time, "Gyro")
		if err != nil {
			return cids, fmt.Errorf("failed due to: %v", err)
		}
		return cids, nil
	}

	return cids, fmt.Errorf("user does not have access to function")
}

func (s *SmartContract) ReadCompassData(ctx contractapi.TransactionContextInterface, v_id int, start_time float64, end_time float64) ([]string, error) {
	objval, ok, err := cid.GetAttributeValue(ctx.GetStub(), "role")
	cids := []string{}
	if err != nil {
		return cids, fmt.Errorf("error while retrieving attributes: %v", err)
	}
	if !ok {
		return cids, fmt.Errorf("client identity does not possess the attribute: %v", err)
	}

	if (objval == "automaker") || (objval == "sensormanufacturer") {
		cids, err = s.ReadFrameData(ctx, v_id, start_time, end_time, "Compass")
		if err != nil {
			return cids, fmt.Errorf("failed due to: %v", err)
		}
		return cids, nil
	}

	return cids, fmt.Errorf("user does not have access to function")
}

func (s *SmartContract) ReadThrottleData(ctx contractapi.TransactionContextInterface, v_id int, start_time float64, end_time float64) ([]string, error) {
	objval, ok, err := cid.GetAttributeValue(ctx.GetStub(), "role")
	cids := []string{}
	if err != nil {
		return cids, fmt.Errorf("error while retrieving attributes: %v", err)
	}
	if !ok {
		return cids, fmt.Errorf("client identity does not possess the attribute: %v", err)
	}

	if (objval == "automaker") || (objval == "actuatorsupplier") {
		cids, err = s.ReadFrameData(ctx, v_id, start_time, end_time, "Throttle")
		if err != nil {
			return cids, fmt.Errorf("failed due to: %v", err)
		}
		return cids, nil
	}

	return cids, fmt.Errorf("user does not have access to function")
}

func (s *SmartContract) ReadSteerData(ctx contractapi.TransactionContextInterface, v_id int, start_time float64, end_time float64) ([]string, error) {
	objval, ok, err := cid.GetAttributeValue(ctx.GetStub(), "role")
	cids := []string{}
	if err != nil {
		return cids, fmt.Errorf("error while retrieving attributes: %v", err)
	}
	if !ok {
		return cids, fmt.Errorf("client identity does not possess the attribute: %v", err)
	}

	if (objval == "automaker") || (objval == "actuatorsupplier") {
		cids, err = s.ReadFrameData(ctx, v_id, start_time, end_time, "Steering")
		if err != nil {
			return cids, fmt.Errorf("failed due to: %v", err)
		}
		return cids, nil
	}

	return cids, fmt.Errorf("user does not have access to function")
}

func (s *SmartContract) ReadBrakeData(ctx contractapi.TransactionContextInterface, v_id int, start_time float64, end_time float64) ([]string, error) {
	objval, ok, err := cid.GetAttributeValue(ctx.GetStub(), "role")
	cids := []string{}
	if err != nil {
		return cids, fmt.Errorf("error while retrieving attributes: %v", err)
	}
	if !ok {
		return cids, fmt.Errorf("client identity does not possess the attribute: %v", err)
	}

	if (objval == "automaker") || (objval == "actuatorsupplier") {
		cids, err = s.ReadFrameData(ctx, v_id, start_time, end_time, "Braking")
		if err != nil {
			return cids, fmt.Errorf("failed due to: %v", err)
		}
		return cids, nil
	}

	return cids, fmt.Errorf("user does not have access to function")
}

func (s *SmartContract) ReadGearData(ctx contractapi.TransactionContextInterface, v_id int, start_time float64, end_time float64) ([]string, error) {
	objval, ok, err := cid.GetAttributeValue(ctx.GetStub(), "role")
	cids := []string{}
	if err != nil {
		return cids, fmt.Errorf("error while retrieving attributes: %v", err)
	}
	if !ok {
		return cids, fmt.Errorf("client identity does not possess the attribute: %v", err)
	}

	if (objval == "automaker") || (objval == "actuatorsupplier") {
		cids, err = s.ReadFrameData(ctx, v_id, start_time, end_time, "Gear")
		if err != nil {
			return cids, fmt.Errorf("failed due to: %v", err)
		}
		return cids, nil
	}

	return cids, fmt.Errorf("user does not have access to function")
}

func (s *SmartContract) ReadHandBrakeData(ctx contractapi.TransactionContextInterface, v_id int, start_time float64, end_time float64) ([]string, error) {
	objval, ok, err := cid.GetAttributeValue(ctx.GetStub(), "role")
	cids := []string{}
	if err != nil {
		return cids, fmt.Errorf("error while retrieving attributes: %v", err)
	}
	if !ok {
		return cids, fmt.Errorf("client identity does not possess the attribute: %v", err)
	}

	if (objval == "automaker") || (objval == "actuatorsupplier") {
		cids, err = s.ReadFrameData(ctx, v_id, start_time, end_time, "HandBrake")
		if err != nil {
			return cids, fmt.Errorf("failed due to: %v", err)
		}
		return cids, nil
	}

	return cids, fmt.Errorf("user does not have access to function")
}

// The main function which will create the chaincode and start it
func main() {
	// NewChaincode creates a new chaincode using contracts passed.
	assetChaincode, err := contractapi.NewChaincode(&SmartContract{})
	if err != nil {
		log.Panicf("Error creating asset-transfer-basic chaincode: %v", err)
	}

	// Start starts the chaincode in the fabric
	if err := assetChaincode.Start(); err != nil {
		log.Panicf("Error starting asset-transfer-basic chaincode: %v", err)
	}
}
