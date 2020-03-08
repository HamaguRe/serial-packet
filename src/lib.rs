// シリアル（UART）通信用のパケット操作ライブラリ
//
// ------------------------------ データ形式 -----------------------------------
// [0xA5, 0x5A, 0x80, 0x04,  0xA0, 0x01, 0x23, 0xAB, 0xCD,   0x44  , 0x04]
//    header  ,  data size, const,      main data        , checksum, footer
// -----------------------------------------------------------------------------
// data size: main dataのByte数． MSBは1にする
// checksum : main dataの全てのバイトのXORをとった値．全てのデータを正常に転送できた場合，
//            受信側では，main dataの全てのバイトとchecksumのXORをとった結果が0になる．


/// パケットを生成
/// 送信できるデータは最大128Byte（データサイズ部が7bitであるため）．
pub fn make_packet(data: &mut Vec<u8>) -> Result<Vec<u8>, &'static str> {
    let data_len = data.len();
    if data_len == 0 {
        return Err("The main data size is 0.");
    } else if data_len > 0x7F {
        return Err("The data size exceeds the maximum value that can be sent in this packet.");
    }

    let mut packet: Vec<u8> = Vec::with_capacity(data_len + 7);
    // Header
    packet.push(0xA5);
    packet.push(0x5A);
    // Data size
    packet.push( (0x80 | (data_len >> 8)) as u8 );
    packet.push( (0xFF & data_len) as u8 );
    // Const
    packet.push(0xA0);
    let checksum = calc_checksum(&data);
    // Set main data
    packet.append(data);
    // Checksum
    packet.push(checksum);
    // Footer
    packet.push(0x04);

    Ok(packet)
}

/// バッファ内を操作してメインデータ部を見つける．
/// packet: 受信したパケットないしはそれが含まれるバッファ．
/// offset: バッファ内のoffset番目から走査を行う．普通はoffset=0とする．
/// return: (main_data, head_pos, tail_pos)
/// main_data: パケット内のメインデータ部
/// head_pos : packet内でヘッダを見つけた位置
/// tail_pos : packet内でのパケットの終端位置
pub fn parser(packet: &Vec<u8>, offset: usize) -> Result<(Vec<u8>, usize, usize), &'static str> {
    let packet_len = packet.len();
    let mut i: usize = offset;

    if packet_len <= offset {
        return Err("Packet size shorter than offset position.");
    } else if packet_len <= 7 {
        return Err("Read data is 7Byte or less.");
    }

    // ヘッダを探す
    let mut header_flag = false;
    let mut head_pos = 0;
    for _ in offset..(packet_len - 1) {
        if packet[i] == 0xA5 {
            if packet[i+1] == 0x5A {
                header_flag = true;
                head_pos = i;
                i += 2;
                break;
            }
        }
        i += 1;
    }

    // ヘッダを読み出せずに最後まで行ってしまった場合の処理
    if header_flag == false {
        return Err("Header does not exist.");
    }

    // バッファオーバーラン対策
    // バッファ内でパケットが途切れている可能性がある
    if (packet_len - i) < 3 {
        return Err("Data size part and constant part do not fit in buffer.");
    }

    // データ長を読む
    let data_size: usize;
    if (packet[i] & 0x80) == 0x80 {
        let tmp_h = ( (packet[i] & 0x7F ) as usize) << 8;
        i += 1;
        let tmp_l = packet[i] as usize;
        data_size = tmp_h | tmp_l;
    } else {
        return Err("Syntax error (The 3rd byte MSB is not 1).");
    }

    // メインデータ長が0ならエラーで返す．
    if data_size == 0 {
        return Err("Main data part is None.");
    }

    // 固定値を見てデータの整合性を確認
    i += 1;
    if packet[i] != 0xA0 {
        return Err("Syntax error (The 5th of the packet is not 0xA0).");
    }

    // バッファオーバーラン対策
    // メインデータ部以降のデータが残りのバッファサイズを超えていた場合の処理
    if (packet_len - i) < (data_size + 3) {
        return Err("The data after the main data section does not fit in the buffer.");
    }

    // メインデータを読む
    let mut main_data: Vec<u8> = Vec::with_capacity(data_size);
    i += 1;
    for j in i..(i + data_size) {
        main_data.push( packet[j] );
    }
    i += data_size;

    // チェックサムで整合性を確認
    if ( calc_checksum(&main_data) ^ packet[i] ) != 0 {
        return Err("Checksum mismatch.");
    }

    // Footer
    i += 1;
    if packet[i] != 0x04 {
        return Err("Footer does not exist.");
    }

    Ok((main_data, head_pos, i))
}

/// データ部の各バイトのXORを計算する
#[inline]
fn calc_checksum(data: &Vec<u8>) -> u8 {
    let mut num = data[0];
    for i in 1..data.len() {
        num ^= data[i];
    }
    num
}


#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn test_parser() {
        // 受信データの前後に関係無い値が存在しても受信データを取り出せる．
        
        // 模擬受信データ
        let packet: Vec<u8> = vec![0x45, 0xA5, 0x22, 0x32,  // 関係ないデータ
            0xA5, 0x5A, 0x80, 0x04, 0xA0, 0x01, 0x23, 0xAB, 0xCD, 0x44, 0x04];
        let (main_data, head, tail) = parser(&packet, 0).unwrap();
    
        assert_eq!(main_data, vec![0x01, 0x23, 0xAB, 0xCD]);
        assert_eq!(head, 4);
        assert_eq!(tail, 14);
    }
}