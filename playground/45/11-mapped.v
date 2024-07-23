// Benchmark "adder" written by ABC on Thu Jul 18 11:07:03 2024

module adder ( 
    \a[0] , \a[10] , \a[11] , \a[12] , \a[13] , \a[14] , \a[15] , \a[16] ,
    \a[17] , \a[18] , \a[19] , \a[1] , \a[20] , \a[21] , \a[22] , \a[23] ,
    \a[24] , \a[25] , \a[26] , \a[27] , \a[28] , \a[29] , \a[2] , \a[30] ,
    \a[31] , \a[3] , \a[4] , \a[5] , \a[6] , \a[7] , \a[8] , \a[9] ,
    \b[0] , \b[10] , \b[11] , \b[12] , \b[13] , \b[14] , \b[15] , \b[16] ,
    \b[17] , \b[18] , \b[19] , \b[1] , \b[20] , \b[21] , \b[22] , \b[23] ,
    \b[24] , \b[25] , \b[26] , \b[27] , \b[28] , \b[29] , \b[2] , \b[30] ,
    \b[3] , \b[4] , \b[5] , \b[6] , \b[7] , \b[8] , \b[9] ,
    \s[0] , \s[10] , \s[11] , \s[12] , \s[13] , \s[14] , \s[15] , \s[16] ,
    \s[17] , \s[18] , \s[19] , \s[1] , \s[20] , \s[21] , \s[22] , \s[23] ,
    \s[24] , \s[25] , \s[26] , \s[27] , \s[28] , \s[29] , \s[2] , \s[30] ,
    \s[31] , \s[3] , \s[4] , \s[5] , \s[6] , \s[7] , \s[8] , \s[9]   );
  input  \a[0] , \a[10] , \a[11] , \a[12] , \a[13] , \a[14] , \a[15] ,
    \a[16] , \a[17] , \a[18] , \a[19] , \a[1] , \a[20] , \a[21] , \a[22] ,
    \a[23] , \a[24] , \a[25] , \a[26] , \a[27] , \a[28] , \a[29] , \a[2] ,
    \a[30] , \a[31] , \a[3] , \a[4] , \a[5] , \a[6] , \a[7] , \a[8] ,
    \a[9] , \b[0] , \b[10] , \b[11] , \b[12] , \b[13] , \b[14] , \b[15] ,
    \b[16] , \b[17] , \b[18] , \b[19] , \b[1] , \b[20] , \b[21] , \b[22] ,
    \b[23] , \b[24] , \b[25] , \b[26] , \b[27] , \b[28] , \b[29] , \b[2] ,
    \b[30] , \b[3] , \b[4] , \b[5] , \b[6] , \b[7] , \b[8] , \b[9] ;
  output \s[0] , \s[10] , \s[11] , \s[12] , \s[13] , \s[14] , \s[15] ,
    \s[16] , \s[17] , \s[18] , \s[19] , \s[1] , \s[20] , \s[21] , \s[22] ,
    \s[23] , \s[24] , \s[25] , \s[26] , \s[27] , \s[28] , \s[29] , \s[2] ,
    \s[30] , \s[31] , \s[3] , \s[4] , \s[5] , \s[6] , \s[7] , \s[8] ,
    \s[9] ;
  wire new_n97, new_n98, new_n99, new_n100, new_n101, new_n102, new_n103,
    new_n104, new_n105, new_n106, new_n107, new_n108, new_n109, new_n110,
    new_n111, new_n112, new_n113, new_n114, new_n115, new_n116, new_n117,
    new_n118, new_n119, new_n120, new_n121, new_n122, new_n123, new_n124,
    new_n125, new_n126, new_n128, new_n129, new_n130, new_n131, new_n132,
    new_n133, new_n134, new_n135, new_n136, new_n138, new_n139, new_n140,
    new_n141, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n158, new_n159, new_n160, new_n162, new_n163, new_n164,
    new_n165, new_n166, new_n167, new_n169, new_n170, new_n171, new_n172,
    new_n173, new_n175, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n188,
    new_n189, new_n190, new_n191, new_n193, new_n194, new_n195, new_n196,
    new_n197, new_n198, new_n199, new_n200, new_n201, new_n202, new_n205,
    new_n206, new_n207, new_n208, new_n209, new_n210, new_n212, new_n213,
    new_n214, new_n215, new_n216, new_n217, new_n218, new_n220, new_n221,
    new_n222, new_n223, new_n224, new_n226, new_n227, new_n228, new_n229,
    new_n230, new_n231, new_n232, new_n233, new_n234, new_n235, new_n237,
    new_n238, new_n239, new_n240, new_n241, new_n243, new_n244, new_n245,
    new_n246, new_n247, new_n248, new_n249, new_n251, new_n252, new_n253,
    new_n254, new_n255, new_n257, new_n258, new_n259, new_n260, new_n261,
    new_n262, new_n263, new_n264, new_n265, new_n266, new_n267, new_n268,
    new_n269, new_n270, new_n271, new_n272, new_n273, new_n274, new_n276,
    new_n277, new_n278, new_n279, new_n280, new_n281, new_n283, new_n284,
    new_n285, new_n286, new_n287, new_n288, new_n289, new_n290, new_n293,
    new_n294, new_n295, new_n296, new_n297, new_n298, new_n299, new_n300,
    new_n302, new_n303, new_n304, new_n305, new_n306, new_n307, new_n308,
    new_n309, new_n312, new_n315, new_n317, new_n319, new_n320;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  inv000aa1d42x5               g001(.a(\a[10] ), .o1(new_n97));
  nor042aa1n02x5               g002(.a(\b[8] ), .b(\a[9] ), .o1(new_n98));
  inv000aa1d42x5               g003(.a(\a[2] ), .o1(new_n99));
  inv000aa1d42x5               g004(.a(\b[1] ), .o1(new_n100));
  nand02aa1n04x5               g005(.a(\b[0] ), .b(\a[1] ), .o1(new_n101));
  oao003aa1n02x5               g006(.a(new_n99), .b(new_n100), .c(new_n101), .carry(new_n102));
  nor022aa1n08x5               g007(.a(\b[3] ), .b(\a[4] ), .o1(new_n103));
  nanp02aa1n04x5               g008(.a(\b[3] ), .b(\a[4] ), .o1(new_n104));
  norp02aa1n12x5               g009(.a(\b[2] ), .b(\a[3] ), .o1(new_n105));
  nand42aa1n03x5               g010(.a(\b[2] ), .b(\a[3] ), .o1(new_n106));
  nano23aa1n03x5               g011(.a(new_n103), .b(new_n105), .c(new_n106), .d(new_n104), .out0(new_n107));
  tech160nm_fioai012aa1n03p5x5 g012(.a(new_n104), .b(new_n105), .c(new_n103), .o1(new_n108));
  aobi12aa1n06x5               g013(.a(new_n108), .b(new_n107), .c(new_n102), .out0(new_n109));
  norp02aa1n09x5               g014(.a(\b[7] ), .b(\a[8] ), .o1(new_n110));
  nand02aa1d16x5               g015(.a(\b[7] ), .b(\a[8] ), .o1(new_n111));
  nor022aa1n16x5               g016(.a(\b[6] ), .b(\a[7] ), .o1(new_n112));
  nand42aa1n04x5               g017(.a(\b[6] ), .b(\a[7] ), .o1(new_n113));
  nano23aa1n02x4               g018(.a(new_n110), .b(new_n112), .c(new_n113), .d(new_n111), .out0(new_n114));
  xnrc02aa1n02x5               g019(.a(\b[5] ), .b(\a[6] ), .out0(new_n115));
  xnrc02aa1n02x5               g020(.a(\b[4] ), .b(\a[5] ), .out0(new_n116));
  nona22aa1n02x4               g021(.a(new_n114), .b(new_n115), .c(new_n116), .out0(new_n117));
  inv000aa1d42x5               g022(.a(\a[6] ), .o1(new_n118));
  inv000aa1d42x5               g023(.a(\b[5] ), .o1(new_n119));
  norp02aa1n02x5               g024(.a(\b[4] ), .b(\a[5] ), .o1(new_n120));
  oao003aa1n02x5               g025(.a(new_n118), .b(new_n119), .c(new_n120), .carry(new_n121));
  tech160nm_fiao0012aa1n02p5x5 g026(.a(new_n110), .b(new_n112), .c(new_n111), .o(new_n122));
  aoi012aa1n02x7               g027(.a(new_n122), .b(new_n114), .c(new_n121), .o1(new_n123));
  oai012aa1n04x7               g028(.a(new_n123), .b(new_n109), .c(new_n117), .o1(new_n124));
  xnrc02aa1n06x5               g029(.a(\b[8] ), .b(\a[9] ), .out0(new_n125));
  aoib12aa1n02x5               g030(.a(new_n98), .b(new_n124), .c(new_n125), .out0(new_n126));
  xorb03aa1n02x5               g031(.a(new_n126), .b(\b[9] ), .c(new_n97), .out0(\s[10] ));
  nor002aa1d32x5               g032(.a(\b[10] ), .b(\a[11] ), .o1(new_n128));
  nand22aa1n04x5               g033(.a(\b[10] ), .b(\a[11] ), .o1(new_n129));
  norb02aa1n02x5               g034(.a(new_n129), .b(new_n128), .out0(new_n130));
  tech160nm_fixnrc02aa1n02p5x5 g035(.a(\b[9] ), .b(\a[10] ), .out0(new_n131));
  nor042aa1n02x5               g036(.a(new_n131), .b(new_n125), .o1(new_n132));
  inv000aa1d42x5               g037(.a(\b[9] ), .o1(new_n133));
  oao003aa1n03x5               g038(.a(new_n97), .b(new_n133), .c(new_n98), .carry(new_n134));
  aoai13aa1n02x5               g039(.a(new_n130), .b(new_n134), .c(new_n124), .d(new_n132), .o1(new_n135));
  aoi112aa1n02x5               g040(.a(new_n130), .b(new_n134), .c(new_n124), .d(new_n132), .o1(new_n136));
  norb02aa1n02x5               g041(.a(new_n135), .b(new_n136), .out0(\s[11] ));
  inv000aa1d42x5               g042(.a(new_n128), .o1(new_n138));
  nor022aa1n08x5               g043(.a(\b[11] ), .b(\a[12] ), .o1(new_n139));
  nand02aa1d04x5               g044(.a(\b[11] ), .b(\a[12] ), .o1(new_n140));
  norb02aa1n02x5               g045(.a(new_n140), .b(new_n139), .out0(new_n141));
  xnbna2aa1n03x5               g046(.a(new_n141), .b(new_n135), .c(new_n138), .out0(\s[12] ));
  tech160nm_fioaoi03aa1n03p5x5 g047(.a(new_n99), .b(new_n100), .c(new_n101), .o1(new_n143));
  nona23aa1n02x4               g048(.a(new_n106), .b(new_n104), .c(new_n103), .d(new_n105), .out0(new_n144));
  oaih12aa1n06x5               g049(.a(new_n108), .b(new_n144), .c(new_n143), .o1(new_n145));
  nona23aa1n06x5               g050(.a(new_n113), .b(new_n111), .c(new_n110), .d(new_n112), .out0(new_n146));
  nor043aa1n03x5               g051(.a(new_n146), .b(new_n115), .c(new_n116), .o1(new_n147));
  oaoi03aa1n02x5               g052(.a(new_n118), .b(new_n119), .c(new_n120), .o1(new_n148));
  oabi12aa1n03x5               g053(.a(new_n122), .b(new_n146), .c(new_n148), .out0(new_n149));
  nona23aa1n03x5               g054(.a(new_n140), .b(new_n129), .c(new_n128), .d(new_n139), .out0(new_n150));
  norp03aa1n02x5               g055(.a(new_n150), .b(new_n131), .c(new_n125), .o1(new_n151));
  aoai13aa1n03x5               g056(.a(new_n151), .b(new_n149), .c(new_n145), .d(new_n147), .o1(new_n152));
  nano23aa1n03x7               g057(.a(new_n128), .b(new_n139), .c(new_n140), .d(new_n129), .out0(new_n153));
  aoi012aa1n02x5               g058(.a(new_n139), .b(new_n128), .c(new_n140), .o1(new_n154));
  aobi12aa1n03x5               g059(.a(new_n154), .b(new_n153), .c(new_n134), .out0(new_n155));
  nanp02aa1n02x5               g060(.a(new_n152), .b(new_n155), .o1(new_n156));
  xorb03aa1n02x5               g061(.a(new_n156), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  nor042aa1d18x5               g062(.a(\b[12] ), .b(\a[13] ), .o1(new_n158));
  nand42aa1d28x5               g063(.a(\b[12] ), .b(\a[13] ), .o1(new_n159));
  aoi012aa1n02x5               g064(.a(new_n158), .b(new_n156), .c(new_n159), .o1(new_n160));
  xnrb03aa1n02x5               g065(.a(new_n160), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  nor042aa1n09x5               g066(.a(\b[13] ), .b(\a[14] ), .o1(new_n162));
  nand02aa1d24x5               g067(.a(\b[13] ), .b(\a[14] ), .o1(new_n163));
  nano23aa1d15x5               g068(.a(new_n158), .b(new_n162), .c(new_n163), .d(new_n159), .out0(new_n164));
  inv000aa1d42x5               g069(.a(new_n164), .o1(new_n165));
  tech160nm_fiaoi012aa1n04x5   g070(.a(new_n162), .b(new_n158), .c(new_n163), .o1(new_n166));
  aoai13aa1n04x5               g071(.a(new_n166), .b(new_n165), .c(new_n152), .d(new_n155), .o1(new_n167));
  xorb03aa1n02x5               g072(.a(new_n167), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  norp02aa1n02x5               g073(.a(\b[14] ), .b(\a[15] ), .o1(new_n169));
  xorc02aa1n12x5               g074(.a(\a[15] ), .b(\b[14] ), .out0(new_n170));
  xorc02aa1n12x5               g075(.a(\a[16] ), .b(\b[15] ), .out0(new_n171));
  aoi112aa1n02x5               g076(.a(new_n171), .b(new_n169), .c(new_n167), .d(new_n170), .o1(new_n172));
  aoai13aa1n02x5               g077(.a(new_n171), .b(new_n169), .c(new_n167), .d(new_n170), .o1(new_n173));
  norb02aa1n03x4               g078(.a(new_n173), .b(new_n172), .out0(\s[16] ));
  nand23aa1d12x5               g079(.a(new_n164), .b(new_n170), .c(new_n171), .o1(new_n175));
  nano22aa1d15x5               g080(.a(new_n175), .b(new_n132), .c(new_n153), .out0(new_n176));
  aoai13aa1n12x5               g081(.a(new_n176), .b(new_n149), .c(new_n145), .d(new_n147), .o1(new_n177));
  oaoi03aa1n03x5               g082(.a(new_n97), .b(new_n133), .c(new_n98), .o1(new_n178));
  oai012aa1n03x5               g083(.a(new_n154), .b(new_n150), .c(new_n178), .o1(new_n179));
  xnrc02aa1n02x5               g084(.a(\b[14] ), .b(\a[15] ), .out0(new_n180));
  xnrc02aa1n02x5               g085(.a(\b[15] ), .b(\a[16] ), .out0(new_n181));
  orn002aa1n02x5               g086(.a(\a[15] ), .b(\b[14] ), .o(new_n182));
  oao003aa1n02x5               g087(.a(\a[16] ), .b(\b[15] ), .c(new_n182), .carry(new_n183));
  oai013aa1n02x5               g088(.a(new_n183), .b(new_n181), .c(new_n180), .d(new_n166), .o1(new_n184));
  aoib12aa1n12x5               g089(.a(new_n184), .b(new_n179), .c(new_n175), .out0(new_n185));
  nanp02aa1n09x5               g090(.a(new_n177), .b(new_n185), .o1(new_n186));
  xorb03aa1n02x5               g091(.a(new_n186), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv000aa1d42x5               g092(.a(\a[18] ), .o1(new_n188));
  inv000aa1d42x5               g093(.a(\a[17] ), .o1(new_n189));
  inv000aa1d42x5               g094(.a(\b[16] ), .o1(new_n190));
  oaoi03aa1n03x5               g095(.a(new_n189), .b(new_n190), .c(new_n186), .o1(new_n191));
  xorb03aa1n02x5               g096(.a(new_n191), .b(\b[17] ), .c(new_n188), .out0(\s[18] ));
  xroi22aa1d06x4               g097(.a(new_n189), .b(\b[16] ), .c(new_n188), .d(\b[17] ), .out0(new_n193));
  nor042aa1n04x5               g098(.a(\b[17] ), .b(\a[18] ), .o1(new_n194));
  aoi112aa1n09x5               g099(.a(\b[16] ), .b(\a[17] ), .c(\a[18] ), .d(\b[17] ), .o1(new_n195));
  nor042aa1n09x5               g100(.a(new_n195), .b(new_n194), .o1(new_n196));
  inv000aa1d42x5               g101(.a(new_n196), .o1(new_n197));
  nor002aa1d32x5               g102(.a(\b[18] ), .b(\a[19] ), .o1(new_n198));
  nand02aa1d12x5               g103(.a(\b[18] ), .b(\a[19] ), .o1(new_n199));
  norb02aa1n12x5               g104(.a(new_n199), .b(new_n198), .out0(new_n200));
  aoai13aa1n06x5               g105(.a(new_n200), .b(new_n197), .c(new_n186), .d(new_n193), .o1(new_n201));
  aoi112aa1n02x5               g106(.a(new_n200), .b(new_n197), .c(new_n186), .d(new_n193), .o1(new_n202));
  norb02aa1n02x7               g107(.a(new_n201), .b(new_n202), .out0(\s[19] ));
  xnrc02aa1n02x5               g108(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor002aa1d24x5               g109(.a(\b[19] ), .b(\a[20] ), .o1(new_n205));
  nand02aa1d16x5               g110(.a(\b[19] ), .b(\a[20] ), .o1(new_n206));
  norb02aa1n12x5               g111(.a(new_n206), .b(new_n205), .out0(new_n207));
  nona22aa1n02x5               g112(.a(new_n201), .b(new_n207), .c(new_n198), .out0(new_n208));
  inv000aa1d42x5               g113(.a(new_n198), .o1(new_n209));
  aobi12aa1n06x5               g114(.a(new_n207), .b(new_n201), .c(new_n209), .out0(new_n210));
  norb02aa1n03x4               g115(.a(new_n208), .b(new_n210), .out0(\s[20] ));
  nona23aa1n12x5               g116(.a(new_n206), .b(new_n199), .c(new_n198), .d(new_n205), .out0(new_n212));
  inv040aa1n06x5               g117(.a(new_n212), .o1(new_n213));
  nanp02aa1n02x5               g118(.a(new_n193), .b(new_n213), .o1(new_n214));
  oai012aa1n12x5               g119(.a(new_n206), .b(new_n205), .c(new_n198), .o1(new_n215));
  oai012aa1n18x5               g120(.a(new_n215), .b(new_n212), .c(new_n196), .o1(new_n216));
  inv000aa1d42x5               g121(.a(new_n216), .o1(new_n217));
  aoai13aa1n04x5               g122(.a(new_n217), .b(new_n214), .c(new_n177), .d(new_n185), .o1(new_n218));
  xorb03aa1n02x5               g123(.a(new_n218), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor042aa1d18x5               g124(.a(\b[20] ), .b(\a[21] ), .o1(new_n220));
  xorc02aa1n12x5               g125(.a(\a[21] ), .b(\b[20] ), .out0(new_n221));
  xorc02aa1n12x5               g126(.a(\a[22] ), .b(\b[21] ), .out0(new_n222));
  aoi112aa1n02x5               g127(.a(new_n220), .b(new_n222), .c(new_n218), .d(new_n221), .o1(new_n223));
  aoai13aa1n03x5               g128(.a(new_n222), .b(new_n220), .c(new_n218), .d(new_n221), .o1(new_n224));
  norb02aa1n02x7               g129(.a(new_n224), .b(new_n223), .out0(\s[22] ));
  nand02aa1n10x5               g130(.a(new_n222), .b(new_n221), .o1(new_n226));
  nanb03aa1n03x5               g131(.a(new_n226), .b(new_n193), .c(new_n213), .out0(new_n227));
  oai112aa1n04x5               g132(.a(new_n200), .b(new_n207), .c(new_n195), .d(new_n194), .o1(new_n228));
  inv000aa1d42x5               g133(.a(\a[22] ), .o1(new_n229));
  inv040aa1d32x5               g134(.a(\b[21] ), .o1(new_n230));
  oao003aa1n03x5               g135(.a(new_n229), .b(new_n230), .c(new_n220), .carry(new_n231));
  inv040aa1n02x5               g136(.a(new_n231), .o1(new_n232));
  aoai13aa1n12x5               g137(.a(new_n232), .b(new_n226), .c(new_n228), .d(new_n215), .o1(new_n233));
  inv000aa1d42x5               g138(.a(new_n233), .o1(new_n234));
  aoai13aa1n04x5               g139(.a(new_n234), .b(new_n227), .c(new_n177), .d(new_n185), .o1(new_n235));
  xorb03aa1n02x5               g140(.a(new_n235), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g141(.a(\b[22] ), .b(\a[23] ), .o1(new_n237));
  tech160nm_fixorc02aa1n02p5x5 g142(.a(\a[23] ), .b(\b[22] ), .out0(new_n238));
  tech160nm_fixorc02aa1n05x5   g143(.a(\a[24] ), .b(\b[23] ), .out0(new_n239));
  aoi112aa1n02x5               g144(.a(new_n237), .b(new_n239), .c(new_n235), .d(new_n238), .o1(new_n240));
  aoai13aa1n03x5               g145(.a(new_n239), .b(new_n237), .c(new_n235), .d(new_n238), .o1(new_n241));
  norb02aa1n02x7               g146(.a(new_n241), .b(new_n240), .out0(\s[24] ));
  and002aa1n02x5               g147(.a(new_n239), .b(new_n238), .o(new_n243));
  nona23aa1n02x4               g148(.a(new_n243), .b(new_n193), .c(new_n226), .d(new_n212), .out0(new_n244));
  inv000aa1d42x5               g149(.a(\a[24] ), .o1(new_n245));
  inv000aa1d42x5               g150(.a(\b[23] ), .o1(new_n246));
  oao003aa1n06x5               g151(.a(new_n245), .b(new_n246), .c(new_n237), .carry(new_n247));
  tech160nm_fiaoi012aa1n05x5   g152(.a(new_n247), .b(new_n233), .c(new_n243), .o1(new_n248));
  aoai13aa1n06x5               g153(.a(new_n248), .b(new_n244), .c(new_n177), .d(new_n185), .o1(new_n249));
  xorb03aa1n02x5               g154(.a(new_n249), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g155(.a(\b[24] ), .b(\a[25] ), .o1(new_n251));
  tech160nm_fixorc02aa1n05x5   g156(.a(\a[25] ), .b(\b[24] ), .out0(new_n252));
  tech160nm_fixorc02aa1n05x5   g157(.a(\a[26] ), .b(\b[25] ), .out0(new_n253));
  aoi112aa1n02x5               g158(.a(new_n251), .b(new_n253), .c(new_n249), .d(new_n252), .o1(new_n254));
  aoai13aa1n03x5               g159(.a(new_n253), .b(new_n251), .c(new_n249), .d(new_n252), .o1(new_n255));
  norb02aa1n02x7               g160(.a(new_n255), .b(new_n254), .out0(\s[26] ));
  oabi12aa1n03x5               g161(.a(new_n184), .b(new_n155), .c(new_n175), .out0(new_n257));
  and002aa1n06x5               g162(.a(new_n253), .b(new_n252), .o(new_n258));
  nano22aa1n03x7               g163(.a(new_n227), .b(new_n243), .c(new_n258), .out0(new_n259));
  aoai13aa1n06x5               g164(.a(new_n259), .b(new_n257), .c(new_n124), .d(new_n176), .o1(new_n260));
  aoai13aa1n09x5               g165(.a(new_n258), .b(new_n247), .c(new_n233), .d(new_n243), .o1(new_n261));
  oai022aa1n02x5               g166(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n262));
  aob012aa1n02x5               g167(.a(new_n262), .b(\b[25] ), .c(\a[26] ), .out0(new_n263));
  xorc02aa1n12x5               g168(.a(\a[27] ), .b(\b[26] ), .out0(new_n264));
  inv000aa1d42x5               g169(.a(new_n264), .o1(new_n265));
  aoi013aa1n03x5               g170(.a(new_n265), .b(new_n260), .c(new_n261), .d(new_n263), .o1(new_n266));
  inv020aa1n02x5               g171(.a(new_n259), .o1(new_n267));
  aoi012aa1n06x5               g172(.a(new_n267), .b(new_n177), .c(new_n185), .o1(new_n268));
  inv000aa1d42x5               g173(.a(new_n226), .o1(new_n269));
  aoai13aa1n06x5               g174(.a(new_n243), .b(new_n231), .c(new_n216), .d(new_n269), .o1(new_n270));
  inv000aa1d42x5               g175(.a(new_n247), .o1(new_n271));
  inv000aa1d42x5               g176(.a(new_n258), .o1(new_n272));
  aoai13aa1n06x5               g177(.a(new_n263), .b(new_n272), .c(new_n270), .d(new_n271), .o1(new_n273));
  norp03aa1n02x5               g178(.a(new_n273), .b(new_n268), .c(new_n264), .o1(new_n274));
  norp02aa1n02x5               g179(.a(new_n266), .b(new_n274), .o1(\s[27] ));
  norp02aa1n02x5               g180(.a(\b[26] ), .b(\a[27] ), .o1(new_n276));
  inv040aa1n03x5               g181(.a(new_n276), .o1(new_n277));
  xnrc02aa1n12x5               g182(.a(\b[27] ), .b(\a[28] ), .out0(new_n278));
  nano22aa1n03x5               g183(.a(new_n266), .b(new_n277), .c(new_n278), .out0(new_n279));
  oaih12aa1n02x5               g184(.a(new_n264), .b(new_n273), .c(new_n268), .o1(new_n280));
  tech160nm_fiaoi012aa1n03p5x5 g185(.a(new_n278), .b(new_n280), .c(new_n277), .o1(new_n281));
  nor002aa1n02x5               g186(.a(new_n281), .b(new_n279), .o1(\s[28] ));
  norb02aa1n15x5               g187(.a(new_n264), .b(new_n278), .out0(new_n283));
  inv000aa1d42x5               g188(.a(new_n283), .o1(new_n284));
  aoi013aa1n03x5               g189(.a(new_n284), .b(new_n260), .c(new_n261), .d(new_n263), .o1(new_n285));
  oao003aa1n02x5               g190(.a(\a[28] ), .b(\b[27] ), .c(new_n277), .carry(new_n286));
  xnrc02aa1n02x5               g191(.a(\b[28] ), .b(\a[29] ), .out0(new_n287));
  nano22aa1n03x5               g192(.a(new_n285), .b(new_n286), .c(new_n287), .out0(new_n288));
  oaih12aa1n02x5               g193(.a(new_n283), .b(new_n273), .c(new_n268), .o1(new_n289));
  tech160nm_fiaoi012aa1n02p5x5 g194(.a(new_n287), .b(new_n289), .c(new_n286), .o1(new_n290));
  norp02aa1n03x5               g195(.a(new_n290), .b(new_n288), .o1(\s[29] ));
  xorb03aa1n02x5               g196(.a(new_n101), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n12x5               g197(.a(new_n264), .b(new_n287), .c(new_n278), .out0(new_n293));
  inv000aa1d42x5               g198(.a(new_n293), .o1(new_n294));
  aoi013aa1n03x5               g199(.a(new_n294), .b(new_n260), .c(new_n261), .d(new_n263), .o1(new_n295));
  oao003aa1n02x5               g200(.a(\a[29] ), .b(\b[28] ), .c(new_n286), .carry(new_n296));
  xnrc02aa1n02x5               g201(.a(\b[29] ), .b(\a[30] ), .out0(new_n297));
  nano22aa1n02x4               g202(.a(new_n295), .b(new_n296), .c(new_n297), .out0(new_n298));
  oaih12aa1n02x5               g203(.a(new_n293), .b(new_n273), .c(new_n268), .o1(new_n299));
  tech160nm_fiaoi012aa1n02p5x5 g204(.a(new_n297), .b(new_n299), .c(new_n296), .o1(new_n300));
  norp02aa1n03x5               g205(.a(new_n300), .b(new_n298), .o1(\s[30] ));
  norb02aa1n02x5               g206(.a(new_n293), .b(new_n297), .out0(new_n302));
  inv020aa1n02x5               g207(.a(new_n302), .o1(new_n303));
  aoi013aa1n03x5               g208(.a(new_n303), .b(new_n260), .c(new_n261), .d(new_n263), .o1(new_n304));
  oao003aa1n02x5               g209(.a(\a[30] ), .b(\b[29] ), .c(new_n296), .carry(new_n305));
  xnrc02aa1n02x5               g210(.a(\b[30] ), .b(\a[31] ), .out0(new_n306));
  nano22aa1n03x5               g211(.a(new_n304), .b(new_n305), .c(new_n306), .out0(new_n307));
  oaih12aa1n02x5               g212(.a(new_n302), .b(new_n273), .c(new_n268), .o1(new_n308));
  tech160nm_fiaoi012aa1n02p5x5 g213(.a(new_n306), .b(new_n308), .c(new_n305), .o1(new_n309));
  norp02aa1n03x5               g214(.a(new_n309), .b(new_n307), .o1(\s[31] ));
  xnrb03aa1n02x5               g215(.a(new_n143), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g216(.a(\a[3] ), .b(\b[2] ), .c(new_n143), .o1(new_n312));
  xorb03aa1n02x5               g217(.a(new_n312), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g218(.a(new_n145), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oaoi03aa1n02x5               g219(.a(\a[5] ), .b(\b[4] ), .c(new_n109), .o1(new_n315));
  xorb03aa1n02x5               g220(.a(new_n315), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  oao003aa1n02x5               g221(.a(new_n118), .b(new_n119), .c(new_n315), .carry(new_n317));
  xorb03aa1n02x5               g222(.a(new_n317), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  inv000aa1d42x5               g223(.a(\a[8] ), .o1(new_n319));
  aoi012aa1n02x5               g224(.a(new_n112), .b(new_n317), .c(new_n113), .o1(new_n320));
  xorb03aa1n02x5               g225(.a(new_n320), .b(\b[7] ), .c(new_n319), .out0(\s[8] ));
  xorb03aa1n02x5               g226(.a(new_n124), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


