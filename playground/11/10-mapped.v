// Benchmark "adder" written by ABC on Wed Jul 17 17:38:46 2024

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
    new_n125, new_n126, new_n127, new_n128, new_n129, new_n130, new_n131,
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n138, new_n140,
    new_n141, new_n142, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n156,
    new_n157, new_n158, new_n160, new_n161, new_n162, new_n163, new_n164,
    new_n166, new_n167, new_n168, new_n169, new_n170, new_n171, new_n172,
    new_n173, new_n175, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n187, new_n188,
    new_n189, new_n190, new_n192, new_n193, new_n194, new_n195, new_n196,
    new_n197, new_n200, new_n201, new_n202, new_n203, new_n204, new_n205,
    new_n206, new_n208, new_n209, new_n210, new_n211, new_n212, new_n213,
    new_n214, new_n215, new_n217, new_n218, new_n219, new_n220, new_n221,
    new_n222, new_n223, new_n225, new_n226, new_n227, new_n228, new_n229,
    new_n230, new_n232, new_n233, new_n234, new_n235, new_n236, new_n238,
    new_n239, new_n240, new_n241, new_n242, new_n243, new_n244, new_n245,
    new_n246, new_n248, new_n249, new_n250, new_n251, new_n252, new_n254,
    new_n255, new_n256, new_n257, new_n258, new_n259, new_n260, new_n261,
    new_n262, new_n263, new_n265, new_n266, new_n267, new_n268, new_n269,
    new_n271, new_n272, new_n273, new_n274, new_n275, new_n276, new_n277,
    new_n278, new_n281, new_n282, new_n283, new_n284, new_n285, new_n286,
    new_n287, new_n288, new_n290, new_n291, new_n292, new_n293, new_n294,
    new_n295, new_n296, new_n297, new_n300, new_n303, new_n304, new_n305,
    new_n307, new_n309;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor002aa1n04x5               g001(.a(\b[9] ), .b(\a[10] ), .o1(new_n97));
  nand42aa1n10x5               g002(.a(\b[9] ), .b(\a[10] ), .o1(new_n98));
  norb02aa1n02x5               g003(.a(new_n98), .b(new_n97), .out0(new_n99));
  norp02aa1n02x5               g004(.a(\b[8] ), .b(\a[9] ), .o1(new_n100));
  nor002aa1n02x5               g005(.a(\b[7] ), .b(\a[8] ), .o1(new_n101));
  nand42aa1n03x5               g006(.a(\b[7] ), .b(\a[8] ), .o1(new_n102));
  norb02aa1n03x4               g007(.a(new_n102), .b(new_n101), .out0(new_n103));
  nor002aa1n02x5               g008(.a(\b[6] ), .b(\a[7] ), .o1(new_n104));
  nand42aa1n03x5               g009(.a(\b[6] ), .b(\a[7] ), .o1(new_n105));
  norb02aa1n06x4               g010(.a(new_n105), .b(new_n104), .out0(new_n106));
  nor042aa1n04x5               g011(.a(\b[5] ), .b(\a[6] ), .o1(new_n107));
  nanp02aa1n02x5               g012(.a(\b[5] ), .b(\a[6] ), .o1(new_n108));
  norp02aa1n04x5               g013(.a(\b[4] ), .b(\a[5] ), .o1(new_n109));
  nanp02aa1n02x5               g014(.a(\b[4] ), .b(\a[5] ), .o1(new_n110));
  nona23aa1n02x4               g015(.a(new_n110), .b(new_n108), .c(new_n107), .d(new_n109), .out0(new_n111));
  nano22aa1n03x7               g016(.a(new_n111), .b(new_n103), .c(new_n106), .out0(new_n112));
  and002aa1n02x5               g017(.a(\b[3] ), .b(\a[4] ), .o(new_n113));
  inv040aa1d32x5               g018(.a(\a[3] ), .o1(new_n114));
  inv000aa1d42x5               g019(.a(\b[2] ), .o1(new_n115));
  tech160nm_finand02aa1n03p5x5 g020(.a(new_n115), .b(new_n114), .o1(new_n116));
  nand02aa1n03x5               g021(.a(\b[2] ), .b(\a[3] ), .o1(new_n117));
  nand22aa1n03x5               g022(.a(new_n116), .b(new_n117), .o1(new_n118));
  nor042aa1n04x5               g023(.a(\b[1] ), .b(\a[2] ), .o1(new_n119));
  nand42aa1n08x5               g024(.a(\b[0] ), .b(\a[1] ), .o1(new_n120));
  nand42aa1n08x5               g025(.a(\b[1] ), .b(\a[2] ), .o1(new_n121));
  aoi012aa1n12x5               g026(.a(new_n119), .b(new_n120), .c(new_n121), .o1(new_n122));
  oa0022aa1n02x5               g027(.a(\a[4] ), .b(\b[3] ), .c(\a[3] ), .d(\b[2] ), .o(new_n123));
  oaoi13aa1n12x5               g028(.a(new_n113), .b(new_n123), .c(new_n122), .d(new_n118), .o1(new_n124));
  aoi112aa1n02x5               g029(.a(\b[6] ), .b(\a[7] ), .c(\a[8] ), .d(\b[7] ), .o1(new_n125));
  aoi112aa1n03x4               g030(.a(\b[4] ), .b(\a[5] ), .c(\a[6] ), .d(\b[5] ), .o1(new_n126));
  oai112aa1n02x7               g031(.a(new_n106), .b(new_n103), .c(new_n126), .d(new_n107), .o1(new_n127));
  nona22aa1n06x5               g032(.a(new_n127), .b(new_n125), .c(new_n101), .out0(new_n128));
  tech160nm_fiao0012aa1n02p5x5 g033(.a(new_n128), .b(new_n124), .c(new_n112), .o(new_n129));
  nand42aa1n03x5               g034(.a(\b[8] ), .b(\a[9] ), .o1(new_n130));
  aoi012aa1n02x5               g035(.a(new_n100), .b(new_n129), .c(new_n130), .o1(new_n131));
  xnrc02aa1n02x5               g036(.a(new_n131), .b(new_n99), .out0(\s[10] ));
  nano23aa1n06x5               g037(.a(new_n97), .b(new_n100), .c(new_n130), .d(new_n98), .out0(new_n133));
  aoai13aa1n02x5               g038(.a(new_n133), .b(new_n128), .c(new_n124), .d(new_n112), .o1(new_n134));
  aoi012aa1n02x5               g039(.a(new_n97), .b(new_n100), .c(new_n98), .o1(new_n135));
  nor002aa1d32x5               g040(.a(\b[10] ), .b(\a[11] ), .o1(new_n136));
  nanp02aa1n24x5               g041(.a(\b[10] ), .b(\a[11] ), .o1(new_n137));
  norb02aa1n15x5               g042(.a(new_n137), .b(new_n136), .out0(new_n138));
  xnbna2aa1n03x5               g043(.a(new_n138), .b(new_n134), .c(new_n135), .out0(\s[11] ));
  inv000aa1d42x5               g044(.a(new_n136), .o1(new_n140));
  inv000aa1d42x5               g045(.a(new_n138), .o1(new_n141));
  aoai13aa1n02x5               g046(.a(new_n140), .b(new_n141), .c(new_n134), .d(new_n135), .o1(new_n142));
  xorb03aa1n02x5               g047(.a(new_n142), .b(\b[11] ), .c(\a[12] ), .out0(\s[12] ));
  nor042aa1n03x5               g048(.a(\b[11] ), .b(\a[12] ), .o1(new_n144));
  aoi112aa1n02x5               g049(.a(\b[10] ), .b(\a[11] ), .c(\a[12] ), .d(\b[11] ), .o1(new_n145));
  aoi112aa1n09x5               g050(.a(\b[8] ), .b(\a[9] ), .c(\a[10] ), .d(\b[9] ), .o1(new_n146));
  nand42aa1n02x5               g051(.a(\b[11] ), .b(\a[12] ), .o1(new_n147));
  norb02aa1n09x5               g052(.a(new_n147), .b(new_n144), .out0(new_n148));
  oai112aa1n06x5               g053(.a(new_n138), .b(new_n148), .c(new_n146), .d(new_n97), .o1(new_n149));
  nona22aa1d18x5               g054(.a(new_n149), .b(new_n145), .c(new_n144), .out0(new_n150));
  inv000aa1d42x5               g055(.a(new_n150), .o1(new_n151));
  and003aa1n02x5               g056(.a(new_n133), .b(new_n148), .c(new_n138), .o(new_n152));
  aoai13aa1n02x5               g057(.a(new_n152), .b(new_n128), .c(new_n124), .d(new_n112), .o1(new_n153));
  nanp02aa1n02x5               g058(.a(new_n153), .b(new_n151), .o1(new_n154));
  xorb03aa1n02x5               g059(.a(new_n154), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  norp02aa1n03x5               g060(.a(\b[12] ), .b(\a[13] ), .o1(new_n156));
  nand42aa1n03x5               g061(.a(\b[12] ), .b(\a[13] ), .o1(new_n157));
  aoi012aa1n02x5               g062(.a(new_n156), .b(new_n154), .c(new_n157), .o1(new_n158));
  xnrb03aa1n02x5               g063(.a(new_n158), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  norp02aa1n03x5               g064(.a(\b[13] ), .b(\a[14] ), .o1(new_n160));
  nand42aa1n03x5               g065(.a(\b[13] ), .b(\a[14] ), .o1(new_n161));
  aoi012aa1n03x5               g066(.a(new_n160), .b(new_n156), .c(new_n161), .o1(new_n162));
  nona23aa1n03x5               g067(.a(new_n161), .b(new_n157), .c(new_n156), .d(new_n160), .out0(new_n163));
  aoai13aa1n03x5               g068(.a(new_n162), .b(new_n163), .c(new_n153), .d(new_n151), .o1(new_n164));
  xorb03aa1n02x5               g069(.a(new_n164), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  nor042aa1n02x5               g070(.a(\b[14] ), .b(\a[15] ), .o1(new_n166));
  nand42aa1n06x5               g071(.a(\b[14] ), .b(\a[15] ), .o1(new_n167));
  nor022aa1n04x5               g072(.a(\b[15] ), .b(\a[16] ), .o1(new_n168));
  nand02aa1n04x5               g073(.a(\b[15] ), .b(\a[16] ), .o1(new_n169));
  nanb02aa1n18x5               g074(.a(new_n168), .b(new_n169), .out0(new_n170));
  inv000aa1d42x5               g075(.a(new_n170), .o1(new_n171));
  aoi112aa1n02x5               g076(.a(new_n171), .b(new_n166), .c(new_n164), .d(new_n167), .o1(new_n172));
  aoai13aa1n02x5               g077(.a(new_n171), .b(new_n166), .c(new_n164), .d(new_n167), .o1(new_n173));
  norb02aa1n03x4               g078(.a(new_n173), .b(new_n172), .out0(\s[16] ));
  nano23aa1n02x4               g079(.a(new_n156), .b(new_n160), .c(new_n161), .d(new_n157), .out0(new_n175));
  nano23aa1n03x7               g080(.a(new_n166), .b(new_n168), .c(new_n169), .d(new_n167), .out0(new_n176));
  nanp02aa1n02x5               g081(.a(new_n176), .b(new_n175), .o1(new_n177));
  nano32aa1n03x7               g082(.a(new_n177), .b(new_n133), .c(new_n138), .d(new_n148), .out0(new_n178));
  aoai13aa1n12x5               g083(.a(new_n178), .b(new_n128), .c(new_n124), .d(new_n112), .o1(new_n179));
  nanb02aa1n02x5               g084(.a(new_n166), .b(new_n167), .out0(new_n180));
  nor043aa1n02x5               g085(.a(new_n163), .b(new_n170), .c(new_n180), .o1(new_n181));
  aoi112aa1n02x5               g086(.a(\b[14] ), .b(\a[15] ), .c(\a[16] ), .d(\b[15] ), .o1(new_n182));
  obai22aa1n02x7               g087(.a(new_n176), .b(new_n162), .c(\a[16] ), .d(\b[15] ), .out0(new_n183));
  aoi112aa1n09x5               g088(.a(new_n183), .b(new_n182), .c(new_n150), .d(new_n181), .o1(new_n184));
  nand22aa1n09x5               g089(.a(new_n184), .b(new_n179), .o1(new_n185));
  xorb03aa1n02x5               g090(.a(new_n185), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv040aa1d32x5               g091(.a(\a[18] ), .o1(new_n187));
  inv000aa1d42x5               g092(.a(\a[17] ), .o1(new_n188));
  inv000aa1d42x5               g093(.a(\b[16] ), .o1(new_n189));
  oaoi03aa1n02x5               g094(.a(new_n188), .b(new_n189), .c(new_n185), .o1(new_n190));
  xorb03aa1n02x5               g095(.a(new_n190), .b(\b[17] ), .c(new_n187), .out0(\s[18] ));
  xroi22aa1d06x4               g096(.a(new_n188), .b(\b[16] ), .c(new_n187), .d(\b[17] ), .out0(new_n192));
  inv000aa1d42x5               g097(.a(new_n192), .o1(new_n193));
  nor042aa1n02x5               g098(.a(\b[17] ), .b(\a[18] ), .o1(new_n194));
  aoi112aa1n09x5               g099(.a(\b[16] ), .b(\a[17] ), .c(\a[18] ), .d(\b[17] ), .o1(new_n195));
  norp02aa1n02x5               g100(.a(new_n195), .b(new_n194), .o1(new_n196));
  aoai13aa1n03x5               g101(.a(new_n196), .b(new_n193), .c(new_n184), .d(new_n179), .o1(new_n197));
  xorb03aa1n02x5               g102(.a(new_n197), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g103(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor042aa1n03x5               g104(.a(\b[18] ), .b(\a[19] ), .o1(new_n200));
  nand02aa1n03x5               g105(.a(\b[18] ), .b(\a[19] ), .o1(new_n201));
  nor042aa1n04x5               g106(.a(\b[19] ), .b(\a[20] ), .o1(new_n202));
  nanp02aa1n04x5               g107(.a(\b[19] ), .b(\a[20] ), .o1(new_n203));
  norb02aa1n06x4               g108(.a(new_n203), .b(new_n202), .out0(new_n204));
  aoi112aa1n02x7               g109(.a(new_n200), .b(new_n204), .c(new_n197), .d(new_n201), .o1(new_n205));
  aoai13aa1n03x5               g110(.a(new_n204), .b(new_n200), .c(new_n197), .d(new_n201), .o1(new_n206));
  norb02aa1n03x4               g111(.a(new_n206), .b(new_n205), .out0(\s[20] ));
  nano23aa1n02x4               g112(.a(new_n200), .b(new_n202), .c(new_n203), .d(new_n201), .out0(new_n208));
  nand02aa1n02x5               g113(.a(new_n192), .b(new_n208), .o1(new_n209));
  aoi112aa1n02x5               g114(.a(\b[18] ), .b(\a[19] ), .c(\a[20] ), .d(\b[19] ), .o1(new_n210));
  norb02aa1n06x4               g115(.a(new_n201), .b(new_n200), .out0(new_n211));
  oai112aa1n06x5               g116(.a(new_n211), .b(new_n204), .c(new_n195), .d(new_n194), .o1(new_n212));
  nona22aa1d18x5               g117(.a(new_n212), .b(new_n210), .c(new_n202), .out0(new_n213));
  inv000aa1d42x5               g118(.a(new_n213), .o1(new_n214));
  aoai13aa1n03x5               g119(.a(new_n214), .b(new_n209), .c(new_n184), .d(new_n179), .o1(new_n215));
  xorb03aa1n02x5               g120(.a(new_n215), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  norp02aa1n02x5               g121(.a(\b[20] ), .b(\a[21] ), .o1(new_n217));
  nanp02aa1n02x5               g122(.a(\b[20] ), .b(\a[21] ), .o1(new_n218));
  nor022aa1n04x5               g123(.a(\b[21] ), .b(\a[22] ), .o1(new_n219));
  nanp02aa1n02x5               g124(.a(\b[21] ), .b(\a[22] ), .o1(new_n220));
  norb02aa1n02x5               g125(.a(new_n220), .b(new_n219), .out0(new_n221));
  aoi112aa1n02x5               g126(.a(new_n217), .b(new_n221), .c(new_n215), .d(new_n218), .o1(new_n222));
  aoai13aa1n03x5               g127(.a(new_n221), .b(new_n217), .c(new_n215), .d(new_n218), .o1(new_n223));
  norb02aa1n03x4               g128(.a(new_n223), .b(new_n222), .out0(\s[22] ));
  nona23aa1n06x5               g129(.a(new_n220), .b(new_n218), .c(new_n217), .d(new_n219), .out0(new_n225));
  inv020aa1n02x5               g130(.a(new_n225), .o1(new_n226));
  aoi112aa1n09x5               g131(.a(\b[20] ), .b(\a[21] ), .c(\a[22] ), .d(\b[21] ), .o1(new_n227));
  aoi112aa1n03x5               g132(.a(new_n219), .b(new_n227), .c(new_n213), .d(new_n226), .o1(new_n228));
  nand23aa1n03x5               g133(.a(new_n192), .b(new_n226), .c(new_n208), .o1(new_n229));
  aoai13aa1n06x5               g134(.a(new_n228), .b(new_n229), .c(new_n184), .d(new_n179), .o1(new_n230));
  xorb03aa1n02x5               g135(.a(new_n230), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g136(.a(\b[22] ), .b(\a[23] ), .o1(new_n232));
  xorc02aa1n12x5               g137(.a(\a[23] ), .b(\b[22] ), .out0(new_n233));
  tech160nm_fixorc02aa1n05x5   g138(.a(\a[24] ), .b(\b[23] ), .out0(new_n234));
  aoi112aa1n03x5               g139(.a(new_n232), .b(new_n234), .c(new_n230), .d(new_n233), .o1(new_n235));
  aoai13aa1n04x5               g140(.a(new_n234), .b(new_n232), .c(new_n230), .d(new_n233), .o1(new_n236));
  norb02aa1n03x4               g141(.a(new_n236), .b(new_n235), .out0(\s[24] ));
  and002aa1n02x5               g142(.a(new_n234), .b(new_n233), .o(new_n238));
  nanb03aa1n03x5               g143(.a(new_n209), .b(new_n238), .c(new_n226), .out0(new_n239));
  norp02aa1n02x5               g144(.a(\b[23] ), .b(\a[24] ), .o1(new_n240));
  aoi112aa1n02x5               g145(.a(\b[22] ), .b(\a[23] ), .c(\a[24] ), .d(\b[23] ), .o1(new_n241));
  oai112aa1n02x5               g146(.a(new_n233), .b(new_n234), .c(new_n227), .d(new_n219), .o1(new_n242));
  nona22aa1n03x5               g147(.a(new_n242), .b(new_n241), .c(new_n240), .out0(new_n243));
  nano22aa1n03x7               g148(.a(new_n225), .b(new_n233), .c(new_n234), .out0(new_n244));
  aoi012aa1n02x5               g149(.a(new_n243), .b(new_n213), .c(new_n244), .o1(new_n245));
  aoai13aa1n06x5               g150(.a(new_n245), .b(new_n239), .c(new_n184), .d(new_n179), .o1(new_n246));
  xorb03aa1n02x5               g151(.a(new_n246), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g152(.a(\b[24] ), .b(\a[25] ), .o1(new_n248));
  xorc02aa1n02x5               g153(.a(\a[25] ), .b(\b[24] ), .out0(new_n249));
  xorc02aa1n02x5               g154(.a(\a[26] ), .b(\b[25] ), .out0(new_n250));
  aoi112aa1n03x5               g155(.a(new_n248), .b(new_n250), .c(new_n246), .d(new_n249), .o1(new_n251));
  aoai13aa1n03x5               g156(.a(new_n250), .b(new_n248), .c(new_n246), .d(new_n249), .o1(new_n252));
  norb02aa1n03x4               g157(.a(new_n252), .b(new_n251), .out0(\s[26] ));
  and002aa1n02x5               g158(.a(new_n250), .b(new_n249), .o(new_n254));
  nano22aa1n12x5               g159(.a(new_n229), .b(new_n238), .c(new_n254), .out0(new_n255));
  norp02aa1n02x5               g160(.a(\b[25] ), .b(\a[26] ), .o1(new_n256));
  aoi112aa1n02x5               g161(.a(\b[24] ), .b(\a[25] ), .c(\a[26] ), .d(\b[25] ), .o1(new_n257));
  aoai13aa1n04x5               g162(.a(new_n254), .b(new_n243), .c(new_n213), .d(new_n244), .o1(new_n258));
  nona22aa1n12x5               g163(.a(new_n258), .b(new_n257), .c(new_n256), .out0(new_n259));
  aoi012aa1n06x5               g164(.a(new_n259), .b(new_n185), .c(new_n255), .o1(new_n260));
  nor042aa1n03x5               g165(.a(\b[26] ), .b(\a[27] ), .o1(new_n261));
  inv000aa1n06x5               g166(.a(new_n261), .o1(new_n262));
  nanp02aa1n02x5               g167(.a(\b[26] ), .b(\a[27] ), .o1(new_n263));
  xnbna2aa1n03x5               g168(.a(new_n260), .b(new_n263), .c(new_n262), .out0(\s[27] ));
  xorc02aa1n12x5               g169(.a(\a[28] ), .b(\b[27] ), .out0(new_n265));
  inv000aa1d42x5               g170(.a(new_n265), .o1(new_n266));
  aoai13aa1n06x5               g171(.a(new_n263), .b(new_n259), .c(new_n185), .d(new_n255), .o1(new_n267));
  tech160nm_fiaoi012aa1n02p5x5 g172(.a(new_n266), .b(new_n267), .c(new_n262), .o1(new_n268));
  nona22aa1n02x5               g173(.a(new_n267), .b(new_n265), .c(new_n261), .out0(new_n269));
  norb02aa1n03x4               g174(.a(new_n269), .b(new_n268), .out0(\s[28] ));
  nano22aa1n03x7               g175(.a(new_n266), .b(new_n262), .c(new_n263), .out0(new_n271));
  aoai13aa1n06x5               g176(.a(new_n271), .b(new_n259), .c(new_n185), .d(new_n255), .o1(new_n272));
  oaoi03aa1n02x5               g177(.a(\a[28] ), .b(\b[27] ), .c(new_n262), .o1(new_n273));
  inv000aa1n03x5               g178(.a(new_n273), .o1(new_n274));
  xorc02aa1n06x5               g179(.a(\a[29] ), .b(\b[28] ), .out0(new_n275));
  inv000aa1d42x5               g180(.a(new_n275), .o1(new_n276));
  aoi012aa1n06x5               g181(.a(new_n276), .b(new_n272), .c(new_n274), .o1(new_n277));
  nona22aa1n06x5               g182(.a(new_n272), .b(new_n273), .c(new_n275), .out0(new_n278));
  norb02aa1n03x4               g183(.a(new_n278), .b(new_n277), .out0(\s[29] ));
  xorb03aa1n02x5               g184(.a(new_n120), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano32aa1n02x4               g185(.a(new_n276), .b(new_n265), .c(new_n263), .d(new_n262), .out0(new_n281));
  aoai13aa1n06x5               g186(.a(new_n281), .b(new_n259), .c(new_n185), .d(new_n255), .o1(new_n282));
  oao003aa1n09x5               g187(.a(\a[29] ), .b(\b[28] ), .c(new_n274), .carry(new_n283));
  xorc02aa1n12x5               g188(.a(\a[30] ), .b(\b[29] ), .out0(new_n284));
  inv000aa1d42x5               g189(.a(new_n284), .o1(new_n285));
  tech160nm_fiaoi012aa1n02p5x5 g190(.a(new_n285), .b(new_n282), .c(new_n283), .o1(new_n286));
  inv000aa1d42x5               g191(.a(new_n283), .o1(new_n287));
  nona22aa1n02x5               g192(.a(new_n282), .b(new_n287), .c(new_n284), .out0(new_n288));
  norb02aa1n03x4               g193(.a(new_n288), .b(new_n286), .out0(\s[30] ));
  xnrc02aa1n02x5               g194(.a(\b[30] ), .b(\a[31] ), .out0(new_n290));
  inv000aa1d42x5               g195(.a(new_n290), .o1(new_n291));
  and003aa1n03x7               g196(.a(new_n271), .b(new_n284), .c(new_n275), .o(new_n292));
  aoai13aa1n06x5               g197(.a(new_n292), .b(new_n259), .c(new_n185), .d(new_n255), .o1(new_n293));
  oaoi03aa1n02x5               g198(.a(\a[30] ), .b(\b[29] ), .c(new_n283), .o1(new_n294));
  nona22aa1n02x5               g199(.a(new_n293), .b(new_n294), .c(new_n291), .out0(new_n295));
  inv000aa1n02x5               g200(.a(new_n294), .o1(new_n296));
  tech160nm_fiaoi012aa1n05x5   g201(.a(new_n290), .b(new_n293), .c(new_n296), .o1(new_n297));
  norb02aa1n03x4               g202(.a(new_n295), .b(new_n297), .out0(\s[31] ));
  xnbna2aa1n03x5               g203(.a(new_n122), .b(new_n116), .c(new_n117), .out0(\s[3] ));
  oaoi03aa1n02x5               g204(.a(\a[3] ), .b(\b[2] ), .c(new_n122), .o1(new_n300));
  xorb03aa1n02x5               g205(.a(new_n300), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g206(.a(new_n124), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  norb02aa1n02x5               g207(.a(new_n108), .b(new_n107), .out0(new_n303));
  oai112aa1n02x5               g208(.a(new_n303), .b(new_n110), .c(new_n124), .d(new_n109), .o1(new_n304));
  oaoi13aa1n02x5               g209(.a(new_n303), .b(new_n110), .c(new_n124), .d(new_n109), .o1(new_n305));
  norb02aa1n02x5               g210(.a(new_n304), .b(new_n305), .out0(\s[6] ));
  norb02aa1n02x5               g211(.a(new_n304), .b(new_n107), .out0(new_n307));
  xnrc02aa1n02x5               g212(.a(new_n307), .b(new_n106), .out0(\s[7] ));
  oaoi03aa1n02x5               g213(.a(\a[7] ), .b(\b[6] ), .c(new_n307), .o1(new_n309));
  xorb03aa1n02x5               g214(.a(new_n309), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xorb03aa1n02x5               g215(.a(new_n129), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


