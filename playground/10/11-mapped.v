// Benchmark "adder" written by ABC on Wed Jul 17 17:08:33 2024

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
    new_n125, new_n126, new_n127, new_n128, new_n129, new_n130, new_n132,
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n138, new_n139,
    new_n140, new_n142, new_n143, new_n144, new_n145, new_n146, new_n147,
    new_n148, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n161, new_n162, new_n163,
    new_n164, new_n165, new_n166, new_n167, new_n169, new_n170, new_n171,
    new_n173, new_n174, new_n175, new_n176, new_n177, new_n178, new_n179,
    new_n180, new_n182, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n193, new_n195,
    new_n196, new_n197, new_n198, new_n200, new_n201, new_n202, new_n203,
    new_n204, new_n205, new_n206, new_n207, new_n210, new_n211, new_n212,
    new_n213, new_n214, new_n215, new_n217, new_n218, new_n219, new_n220,
    new_n221, new_n222, new_n223, new_n224, new_n225, new_n227, new_n228,
    new_n229, new_n230, new_n231, new_n233, new_n234, new_n235, new_n236,
    new_n237, new_n238, new_n239, new_n240, new_n241, new_n243, new_n244,
    new_n245, new_n246, new_n247, new_n248, new_n250, new_n251, new_n252,
    new_n253, new_n254, new_n255, new_n256, new_n257, new_n258, new_n259,
    new_n260, new_n262, new_n263, new_n264, new_n265, new_n266, new_n268,
    new_n269, new_n270, new_n271, new_n272, new_n273, new_n274, new_n275,
    new_n276, new_n277, new_n278, new_n279, new_n280, new_n282, new_n283,
    new_n284, new_n285, new_n286, new_n287, new_n288, new_n289, new_n290,
    new_n291, new_n293, new_n294, new_n295, new_n296, new_n297, new_n298,
    new_n299, new_n302, new_n303, new_n304, new_n305, new_n306, new_n307,
    new_n308, new_n310, new_n311, new_n312, new_n313, new_n314, new_n315,
    new_n316, new_n319, new_n322, new_n324, new_n325, new_n326, new_n328;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor042aa1n09x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  inv040aa1n03x5               g002(.a(new_n97), .o1(new_n98));
  and002aa1n09x5               g003(.a(\b[5] ), .b(\a[6] ), .o(new_n99));
  inv000aa1d42x5               g004(.a(\a[5] ), .o1(new_n100));
  inv000aa1d42x5               g005(.a(\a[6] ), .o1(new_n101));
  inv000aa1d42x5               g006(.a(\b[4] ), .o1(new_n102));
  aboi22aa1n02x7               g007(.a(\b[5] ), .b(new_n101), .c(new_n100), .d(new_n102), .out0(new_n103));
  norp02aa1n04x5               g008(.a(\b[7] ), .b(\a[8] ), .o1(new_n104));
  nanp02aa1n04x5               g009(.a(\b[7] ), .b(\a[8] ), .o1(new_n105));
  nor022aa1n16x5               g010(.a(\b[6] ), .b(\a[7] ), .o1(new_n106));
  nanp02aa1n04x5               g011(.a(\b[6] ), .b(\a[7] ), .o1(new_n107));
  nona23aa1n09x5               g012(.a(new_n107), .b(new_n105), .c(new_n104), .d(new_n106), .out0(new_n108));
  aoi012aa1n02x5               g013(.a(new_n104), .b(new_n106), .c(new_n105), .o1(new_n109));
  oai013aa1n06x5               g014(.a(new_n109), .b(new_n108), .c(new_n99), .d(new_n103), .o1(new_n110));
  nanp02aa1n02x5               g015(.a(\b[1] ), .b(\a[2] ), .o1(new_n111));
  nanp02aa1n02x5               g016(.a(\b[0] ), .b(\a[1] ), .o1(new_n112));
  nor002aa1n02x5               g017(.a(\b[1] ), .b(\a[2] ), .o1(new_n113));
  tech160nm_fioai012aa1n05x5   g018(.a(new_n111), .b(new_n113), .c(new_n112), .o1(new_n114));
  nor022aa1n06x5               g019(.a(\b[3] ), .b(\a[4] ), .o1(new_n115));
  nand22aa1n03x5               g020(.a(\b[3] ), .b(\a[4] ), .o1(new_n116));
  nor022aa1n16x5               g021(.a(\b[2] ), .b(\a[3] ), .o1(new_n117));
  nanp02aa1n02x5               g022(.a(\b[2] ), .b(\a[3] ), .o1(new_n118));
  nona23aa1n06x5               g023(.a(new_n118), .b(new_n116), .c(new_n115), .d(new_n117), .out0(new_n119));
  oai012aa1n02x5               g024(.a(new_n116), .b(new_n117), .c(new_n115), .o1(new_n120));
  oai012aa1n12x5               g025(.a(new_n120), .b(new_n119), .c(new_n114), .o1(new_n121));
  xnrc02aa1n02x5               g026(.a(\b[5] ), .b(\a[6] ), .out0(new_n122));
  xnrc02aa1n02x5               g027(.a(\b[4] ), .b(\a[5] ), .out0(new_n123));
  nor043aa1n06x5               g028(.a(new_n108), .b(new_n122), .c(new_n123), .o1(new_n124));
  nand42aa1n06x5               g029(.a(\b[8] ), .b(\a[9] ), .o1(new_n125));
  norb02aa1n02x5               g030(.a(new_n125), .b(new_n97), .out0(new_n126));
  aoai13aa1n03x5               g031(.a(new_n126), .b(new_n110), .c(new_n124), .d(new_n121), .o1(new_n127));
  nor042aa1n04x5               g032(.a(\b[9] ), .b(\a[10] ), .o1(new_n128));
  nand42aa1n06x5               g033(.a(\b[9] ), .b(\a[10] ), .o1(new_n129));
  norb02aa1n02x5               g034(.a(new_n129), .b(new_n128), .out0(new_n130));
  xnbna2aa1n03x5               g035(.a(new_n130), .b(new_n127), .c(new_n98), .out0(\s[10] ));
  aobi12aa1n03x5               g036(.a(new_n130), .b(new_n127), .c(new_n98), .out0(new_n132));
  nand22aa1n12x5               g037(.a(\b[10] ), .b(\a[11] ), .o1(new_n133));
  nor002aa1d32x5               g038(.a(\b[10] ), .b(\a[11] ), .o1(new_n134));
  norb02aa1n02x5               g039(.a(new_n133), .b(new_n134), .out0(new_n135));
  oaoi03aa1n02x5               g040(.a(\a[10] ), .b(\b[9] ), .c(new_n98), .o1(new_n136));
  tech160nm_fioai012aa1n04x5   g041(.a(new_n135), .b(new_n132), .c(new_n136), .o1(new_n137));
  nanb02aa1d36x5               g042(.a(new_n134), .b(new_n133), .out0(new_n138));
  tech160nm_fioai012aa1n04x5   g043(.a(new_n129), .b(new_n128), .c(new_n97), .o1(new_n139));
  nano22aa1n02x4               g044(.a(new_n132), .b(new_n138), .c(new_n139), .out0(new_n140));
  norb02aa1n02x5               g045(.a(new_n137), .b(new_n140), .out0(\s[11] ));
  nor002aa1n12x5               g046(.a(\b[11] ), .b(\a[12] ), .o1(new_n142));
  nand02aa1n10x5               g047(.a(\b[11] ), .b(\a[12] ), .o1(new_n143));
  norb02aa1n02x5               g048(.a(new_n143), .b(new_n142), .out0(new_n144));
  nona22aa1n02x5               g049(.a(new_n137), .b(new_n144), .c(new_n134), .out0(new_n145));
  inv000aa1d42x5               g050(.a(new_n134), .o1(new_n146));
  nanb02aa1d36x5               g051(.a(new_n142), .b(new_n143), .out0(new_n147));
  tech160nm_fiaoi012aa1n04x5   g052(.a(new_n147), .b(new_n137), .c(new_n146), .o1(new_n148));
  norb02aa1n03x4               g053(.a(new_n145), .b(new_n148), .out0(\s[12] ));
  nano23aa1d15x5               g054(.a(new_n97), .b(new_n128), .c(new_n129), .d(new_n125), .out0(new_n150));
  nona22aa1d36x5               g055(.a(new_n150), .b(new_n147), .c(new_n138), .out0(new_n151));
  inv000aa1d42x5               g056(.a(new_n151), .o1(new_n152));
  aoai13aa1n06x5               g057(.a(new_n152), .b(new_n110), .c(new_n124), .d(new_n121), .o1(new_n153));
  aoi012aa1d24x5               g058(.a(new_n142), .b(new_n134), .c(new_n143), .o1(new_n154));
  oai013aa1n09x5               g059(.a(new_n154), .b(new_n139), .c(new_n138), .d(new_n147), .o1(new_n155));
  inv000aa1d42x5               g060(.a(new_n155), .o1(new_n156));
  nand42aa1n03x5               g061(.a(\b[12] ), .b(\a[13] ), .o1(new_n157));
  nor002aa1n02x5               g062(.a(\b[12] ), .b(\a[13] ), .o1(new_n158));
  norb02aa1n06x4               g063(.a(new_n157), .b(new_n158), .out0(new_n159));
  xnbna2aa1n03x5               g064(.a(new_n159), .b(new_n153), .c(new_n156), .out0(\s[13] ));
  inv000aa1d42x5               g065(.a(\a[13] ), .o1(new_n161));
  inv000aa1d42x5               g066(.a(\b[12] ), .o1(new_n162));
  nanp02aa1n02x5               g067(.a(new_n153), .b(new_n156), .o1(new_n163));
  oaoi03aa1n02x5               g068(.a(new_n161), .b(new_n162), .c(new_n163), .o1(new_n164));
  nor022aa1n16x5               g069(.a(\b[13] ), .b(\a[14] ), .o1(new_n165));
  nanp02aa1n04x5               g070(.a(\b[13] ), .b(\a[14] ), .o1(new_n166));
  norb02aa1n12x5               g071(.a(new_n166), .b(new_n165), .out0(new_n167));
  xnrc02aa1n02x5               g072(.a(new_n164), .b(new_n167), .out0(\s[14] ));
  nona23aa1n02x4               g073(.a(new_n157), .b(new_n166), .c(new_n165), .d(new_n158), .out0(new_n169));
  aoai13aa1n02x5               g074(.a(new_n166), .b(new_n165), .c(new_n161), .d(new_n162), .o1(new_n170));
  aoai13aa1n06x5               g075(.a(new_n170), .b(new_n169), .c(new_n153), .d(new_n156), .o1(new_n171));
  xorb03aa1n02x5               g076(.a(new_n171), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  nor042aa1n02x5               g077(.a(\b[14] ), .b(\a[15] ), .o1(new_n173));
  tech160nm_finand02aa1n03p5x5 g078(.a(\b[14] ), .b(\a[15] ), .o1(new_n174));
  norb02aa1n02x5               g079(.a(new_n174), .b(new_n173), .out0(new_n175));
  nor002aa1n12x5               g080(.a(\b[15] ), .b(\a[16] ), .o1(new_n176));
  nand42aa1n03x5               g081(.a(\b[15] ), .b(\a[16] ), .o1(new_n177));
  norb02aa1n02x5               g082(.a(new_n177), .b(new_n176), .out0(new_n178));
  aoi112aa1n02x5               g083(.a(new_n178), .b(new_n173), .c(new_n171), .d(new_n175), .o1(new_n179));
  aoai13aa1n03x5               g084(.a(new_n178), .b(new_n173), .c(new_n171), .d(new_n174), .o1(new_n180));
  norb02aa1n02x7               g085(.a(new_n180), .b(new_n179), .out0(\s[16] ));
  nano23aa1n09x5               g086(.a(new_n176), .b(new_n173), .c(new_n177), .d(new_n174), .out0(new_n182));
  nand23aa1d12x5               g087(.a(new_n182), .b(new_n159), .c(new_n167), .o1(new_n183));
  nor042aa1d18x5               g088(.a(new_n183), .b(new_n151), .o1(new_n184));
  aoai13aa1n12x5               g089(.a(new_n184), .b(new_n110), .c(new_n124), .d(new_n121), .o1(new_n185));
  inv000aa1d42x5               g090(.a(new_n176), .o1(new_n186));
  aoi112aa1n06x5               g091(.a(\b[14] ), .b(\a[15] ), .c(\a[16] ), .d(\b[15] ), .o1(new_n187));
  inv000aa1d42x5               g092(.a(new_n187), .o1(new_n188));
  nanb03aa1n03x5               g093(.a(new_n170), .b(new_n178), .c(new_n175), .out0(new_n189));
  nand23aa1n03x5               g094(.a(new_n136), .b(new_n135), .c(new_n144), .o1(new_n190));
  aoi012aa1n03x5               g095(.a(new_n183), .b(new_n190), .c(new_n154), .o1(new_n191));
  nano32aa1n03x7               g096(.a(new_n191), .b(new_n189), .c(new_n188), .d(new_n186), .out0(new_n192));
  nanp02aa1n12x5               g097(.a(new_n192), .b(new_n185), .o1(new_n193));
  xorb03aa1n02x5               g098(.a(new_n193), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv000aa1d42x5               g099(.a(\a[18] ), .o1(new_n195));
  inv000aa1d42x5               g100(.a(\a[17] ), .o1(new_n196));
  inv000aa1d42x5               g101(.a(\b[16] ), .o1(new_n197));
  oaoi03aa1n02x5               g102(.a(new_n196), .b(new_n197), .c(new_n193), .o1(new_n198));
  xorb03aa1n02x5               g103(.a(new_n198), .b(\b[17] ), .c(new_n195), .out0(\s[18] ));
  xroi22aa1d06x4               g104(.a(new_n196), .b(\b[16] ), .c(new_n195), .d(\b[17] ), .out0(new_n200));
  nanp02aa1n02x5               g105(.a(new_n197), .b(new_n196), .o1(new_n201));
  oaoi03aa1n02x5               g106(.a(\a[18] ), .b(\b[17] ), .c(new_n201), .o1(new_n202));
  nand02aa1d04x5               g107(.a(\b[18] ), .b(\a[19] ), .o1(new_n203));
  nor042aa1n04x5               g108(.a(\b[18] ), .b(\a[19] ), .o1(new_n204));
  norb02aa1n02x5               g109(.a(new_n203), .b(new_n204), .out0(new_n205));
  aoai13aa1n06x5               g110(.a(new_n205), .b(new_n202), .c(new_n193), .d(new_n200), .o1(new_n206));
  aoi112aa1n02x5               g111(.a(new_n205), .b(new_n202), .c(new_n193), .d(new_n200), .o1(new_n207));
  norb02aa1n02x5               g112(.a(new_n206), .b(new_n207), .out0(\s[19] ));
  xnrc02aa1n02x5               g113(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor042aa1n03x5               g114(.a(\b[19] ), .b(\a[20] ), .o1(new_n210));
  nand22aa1n04x5               g115(.a(\b[19] ), .b(\a[20] ), .o1(new_n211));
  norb02aa1n02x5               g116(.a(new_n211), .b(new_n210), .out0(new_n212));
  nona22aa1n02x5               g117(.a(new_n206), .b(new_n212), .c(new_n204), .out0(new_n213));
  inv000aa1d42x5               g118(.a(new_n212), .o1(new_n214));
  oaoi13aa1n06x5               g119(.a(new_n214), .b(new_n206), .c(\a[19] ), .d(\b[18] ), .o1(new_n215));
  norb02aa1n03x4               g120(.a(new_n213), .b(new_n215), .out0(\s[20] ));
  nano23aa1n03x7               g121(.a(new_n210), .b(new_n204), .c(new_n211), .d(new_n203), .out0(new_n217));
  nanp02aa1n02x5               g122(.a(new_n200), .b(new_n217), .o1(new_n218));
  oai022aa1n02x5               g123(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n219));
  oaib12aa1n02x5               g124(.a(new_n219), .b(new_n195), .c(\b[17] ), .out0(new_n220));
  nona23aa1n09x5               g125(.a(new_n203), .b(new_n211), .c(new_n210), .d(new_n204), .out0(new_n221));
  aoi012aa1n09x5               g126(.a(new_n210), .b(new_n204), .c(new_n211), .o1(new_n222));
  oai012aa1n12x5               g127(.a(new_n222), .b(new_n221), .c(new_n220), .o1(new_n223));
  inv000aa1d42x5               g128(.a(new_n223), .o1(new_n224));
  aoai13aa1n06x5               g129(.a(new_n224), .b(new_n218), .c(new_n192), .d(new_n185), .o1(new_n225));
  xorb03aa1n02x5               g130(.a(new_n225), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor002aa1n02x5               g131(.a(\b[20] ), .b(\a[21] ), .o1(new_n227));
  xorc02aa1n02x5               g132(.a(\a[21] ), .b(\b[20] ), .out0(new_n228));
  xorc02aa1n02x5               g133(.a(\a[22] ), .b(\b[21] ), .out0(new_n229));
  aoi112aa1n02x5               g134(.a(new_n227), .b(new_n229), .c(new_n225), .d(new_n228), .o1(new_n230));
  aoai13aa1n03x5               g135(.a(new_n229), .b(new_n227), .c(new_n225), .d(new_n228), .o1(new_n231));
  norb02aa1n02x7               g136(.a(new_n231), .b(new_n230), .out0(\s[22] ));
  inv000aa1d42x5               g137(.a(\a[21] ), .o1(new_n233));
  inv000aa1d42x5               g138(.a(\a[22] ), .o1(new_n234));
  xroi22aa1d06x4               g139(.a(new_n233), .b(\b[20] ), .c(new_n234), .d(\b[21] ), .out0(new_n235));
  nanp03aa1n02x5               g140(.a(new_n235), .b(new_n200), .c(new_n217), .o1(new_n236));
  inv000aa1d42x5               g141(.a(\b[21] ), .o1(new_n237));
  oaoi03aa1n12x5               g142(.a(new_n234), .b(new_n237), .c(new_n227), .o1(new_n238));
  inv000aa1d42x5               g143(.a(new_n238), .o1(new_n239));
  aoi012aa1n02x5               g144(.a(new_n239), .b(new_n223), .c(new_n235), .o1(new_n240));
  aoai13aa1n04x5               g145(.a(new_n240), .b(new_n236), .c(new_n192), .d(new_n185), .o1(new_n241));
  xorb03aa1n02x5               g146(.a(new_n241), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  nor042aa1n03x5               g147(.a(\b[22] ), .b(\a[23] ), .o1(new_n243));
  and002aa1n12x5               g148(.a(\b[22] ), .b(\a[23] ), .o(new_n244));
  inv000aa1d42x5               g149(.a(new_n244), .o1(new_n245));
  xorc02aa1n02x5               g150(.a(\a[24] ), .b(\b[23] ), .out0(new_n246));
  aoi112aa1n02x5               g151(.a(new_n243), .b(new_n246), .c(new_n241), .d(new_n245), .o1(new_n247));
  aoai13aa1n03x5               g152(.a(new_n246), .b(new_n243), .c(new_n241), .d(new_n245), .o1(new_n248));
  norb02aa1n02x7               g153(.a(new_n248), .b(new_n247), .out0(\s[24] ));
  nano22aa1n09x5               g154(.a(new_n243), .b(new_n246), .c(new_n245), .out0(new_n250));
  inv000aa1n02x5               g155(.a(new_n250), .o1(new_n251));
  nor042aa1n02x5               g156(.a(new_n236), .b(new_n251), .o1(new_n252));
  inv000aa1n02x5               g157(.a(new_n222), .o1(new_n253));
  aoai13aa1n02x5               g158(.a(new_n235), .b(new_n253), .c(new_n217), .d(new_n202), .o1(new_n254));
  inv000aa1d42x5               g159(.a(new_n243), .o1(new_n255));
  oao003aa1n02x5               g160(.a(\a[24] ), .b(\b[23] ), .c(new_n255), .carry(new_n256));
  aoai13aa1n06x5               g161(.a(new_n256), .b(new_n251), .c(new_n254), .d(new_n238), .o1(new_n257));
  tech160nm_fixorc02aa1n05x5   g162(.a(\a[25] ), .b(\b[24] ), .out0(new_n258));
  aoai13aa1n06x5               g163(.a(new_n258), .b(new_n257), .c(new_n193), .d(new_n252), .o1(new_n259));
  aoi112aa1n02x5               g164(.a(new_n258), .b(new_n257), .c(new_n193), .d(new_n252), .o1(new_n260));
  norb02aa1n02x5               g165(.a(new_n259), .b(new_n260), .out0(\s[25] ));
  norp02aa1n02x5               g166(.a(\b[24] ), .b(\a[25] ), .o1(new_n262));
  xorc02aa1n02x5               g167(.a(\a[26] ), .b(\b[25] ), .out0(new_n263));
  nona22aa1n06x5               g168(.a(new_n259), .b(new_n263), .c(new_n262), .out0(new_n264));
  inv000aa1n02x5               g169(.a(new_n262), .o1(new_n265));
  aobi12aa1n06x5               g170(.a(new_n263), .b(new_n259), .c(new_n265), .out0(new_n266));
  norb02aa1n03x4               g171(.a(new_n264), .b(new_n266), .out0(\s[26] ));
  nanp02aa1n02x5               g172(.a(new_n121), .b(new_n124), .o1(new_n268));
  nanb02aa1n02x5               g173(.a(new_n110), .b(new_n268), .out0(new_n269));
  nano22aa1n02x4               g174(.a(new_n170), .b(new_n175), .c(new_n178), .out0(new_n270));
  nanb02aa1n06x5               g175(.a(new_n183), .b(new_n155), .out0(new_n271));
  nona32aa1n03x5               g176(.a(new_n271), .b(new_n270), .c(new_n187), .d(new_n176), .out0(new_n272));
  and002aa1n06x5               g177(.a(new_n263), .b(new_n258), .o(new_n273));
  nano22aa1n06x5               g178(.a(new_n236), .b(new_n250), .c(new_n273), .out0(new_n274));
  aoai13aa1n06x5               g179(.a(new_n274), .b(new_n272), .c(new_n269), .d(new_n184), .o1(new_n275));
  oao003aa1n02x5               g180(.a(\a[26] ), .b(\b[25] ), .c(new_n265), .carry(new_n276));
  aobi12aa1n06x5               g181(.a(new_n276), .b(new_n257), .c(new_n273), .out0(new_n277));
  norp02aa1n02x5               g182(.a(\b[26] ), .b(\a[27] ), .o1(new_n278));
  nanp02aa1n02x5               g183(.a(\b[26] ), .b(\a[27] ), .o1(new_n279));
  norb02aa1n02x5               g184(.a(new_n279), .b(new_n278), .out0(new_n280));
  xnbna2aa1n03x5               g185(.a(new_n280), .b(new_n277), .c(new_n275), .out0(\s[27] ));
  inv000aa1n06x5               g186(.a(new_n278), .o1(new_n282));
  xnrc02aa1n02x5               g187(.a(\b[27] ), .b(\a[28] ), .out0(new_n283));
  aobi12aa1n02x5               g188(.a(new_n279), .b(new_n277), .c(new_n275), .out0(new_n284));
  nano22aa1n03x5               g189(.a(new_n284), .b(new_n282), .c(new_n283), .out0(new_n285));
  aobi12aa1n06x5               g190(.a(new_n274), .b(new_n192), .c(new_n185), .out0(new_n286));
  aoai13aa1n06x5               g191(.a(new_n250), .b(new_n239), .c(new_n223), .d(new_n235), .o1(new_n287));
  inv000aa1d42x5               g192(.a(new_n273), .o1(new_n288));
  aoai13aa1n06x5               g193(.a(new_n276), .b(new_n288), .c(new_n287), .d(new_n256), .o1(new_n289));
  oaih12aa1n02x5               g194(.a(new_n279), .b(new_n289), .c(new_n286), .o1(new_n290));
  tech160nm_fiaoi012aa1n02p5x5 g195(.a(new_n283), .b(new_n290), .c(new_n282), .o1(new_n291));
  norp02aa1n03x5               g196(.a(new_n291), .b(new_n285), .o1(\s[28] ));
  nano22aa1n02x4               g197(.a(new_n283), .b(new_n282), .c(new_n279), .out0(new_n293));
  oaih12aa1n02x5               g198(.a(new_n293), .b(new_n289), .c(new_n286), .o1(new_n294));
  oao003aa1n02x5               g199(.a(\a[28] ), .b(\b[27] ), .c(new_n282), .carry(new_n295));
  xnrc02aa1n02x5               g200(.a(\b[28] ), .b(\a[29] ), .out0(new_n296));
  tech160nm_fiaoi012aa1n02p5x5 g201(.a(new_n296), .b(new_n294), .c(new_n295), .o1(new_n297));
  aobi12aa1n02x5               g202(.a(new_n293), .b(new_n277), .c(new_n275), .out0(new_n298));
  nano22aa1n03x5               g203(.a(new_n298), .b(new_n295), .c(new_n296), .out0(new_n299));
  norp02aa1n03x5               g204(.a(new_n297), .b(new_n299), .o1(\s[29] ));
  xorb03aa1n02x5               g205(.a(new_n112), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g206(.a(new_n280), .b(new_n296), .c(new_n283), .out0(new_n302));
  oaih12aa1n02x5               g207(.a(new_n302), .b(new_n289), .c(new_n286), .o1(new_n303));
  oao003aa1n02x5               g208(.a(\a[29] ), .b(\b[28] ), .c(new_n295), .carry(new_n304));
  xnrc02aa1n02x5               g209(.a(\b[29] ), .b(\a[30] ), .out0(new_n305));
  tech160nm_fiaoi012aa1n02p5x5 g210(.a(new_n305), .b(new_n303), .c(new_n304), .o1(new_n306));
  aobi12aa1n02x5               g211(.a(new_n302), .b(new_n277), .c(new_n275), .out0(new_n307));
  nano22aa1n03x5               g212(.a(new_n307), .b(new_n304), .c(new_n305), .out0(new_n308));
  norp02aa1n03x5               g213(.a(new_n306), .b(new_n308), .o1(\s[30] ));
  norb03aa1n02x5               g214(.a(new_n293), .b(new_n305), .c(new_n296), .out0(new_n310));
  aobi12aa1n02x5               g215(.a(new_n310), .b(new_n277), .c(new_n275), .out0(new_n311));
  oao003aa1n02x5               g216(.a(\a[30] ), .b(\b[29] ), .c(new_n304), .carry(new_n312));
  xnrc02aa1n02x5               g217(.a(\b[30] ), .b(\a[31] ), .out0(new_n313));
  nano22aa1n03x5               g218(.a(new_n311), .b(new_n312), .c(new_n313), .out0(new_n314));
  oaih12aa1n02x5               g219(.a(new_n310), .b(new_n289), .c(new_n286), .o1(new_n315));
  tech160nm_fiaoi012aa1n02p5x5 g220(.a(new_n313), .b(new_n315), .c(new_n312), .o1(new_n316));
  norp02aa1n03x5               g221(.a(new_n316), .b(new_n314), .o1(\s[31] ));
  xnrb03aa1n02x5               g222(.a(new_n114), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g223(.a(\a[3] ), .b(\b[2] ), .c(new_n114), .o1(new_n319));
  xorb03aa1n02x5               g224(.a(new_n319), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g225(.a(new_n121), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oaoi03aa1n02x5               g226(.a(new_n100), .b(new_n102), .c(new_n121), .o1(new_n322));
  xorb03aa1n02x5               g227(.a(new_n322), .b(\b[5] ), .c(new_n101), .out0(\s[6] ));
  inv000aa1d42x5               g228(.a(new_n99), .o1(new_n324));
  nanb02aa1n02x5               g229(.a(new_n106), .b(new_n107), .out0(new_n325));
  nanb02aa1n02x5               g230(.a(new_n122), .b(new_n322), .out0(new_n326));
  xnbna2aa1n03x5               g231(.a(new_n325), .b(new_n326), .c(new_n324), .out0(\s[7] ));
  aoi013aa1n02x4               g232(.a(new_n106), .b(new_n326), .c(new_n107), .d(new_n324), .o1(new_n328));
  xnrb03aa1n02x5               g233(.a(new_n328), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xorb03aa1n02x5               g234(.a(new_n269), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


