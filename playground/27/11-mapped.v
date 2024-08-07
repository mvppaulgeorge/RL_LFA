// Benchmark "adder" written by ABC on Thu Jul 18 01:52:05 2024

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
    new_n125, new_n127, new_n128, new_n129, new_n130, new_n131, new_n132,
    new_n133, new_n134, new_n135, new_n136, new_n138, new_n139, new_n140,
    new_n141, new_n142, new_n143, new_n144, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n158, new_n159, new_n160, new_n161, new_n162, new_n164,
    new_n165, new_n166, new_n168, new_n169, new_n170, new_n171, new_n172,
    new_n173, new_n174, new_n175, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n187, new_n188,
    new_n189, new_n190, new_n192, new_n193, new_n194, new_n195, new_n196,
    new_n199, new_n200, new_n201, new_n202, new_n203, new_n204, new_n205,
    new_n206, new_n207, new_n208, new_n209, new_n210, new_n211, new_n213,
    new_n214, new_n215, new_n216, new_n217, new_n218, new_n219, new_n220,
    new_n222, new_n223, new_n224, new_n225, new_n226, new_n227, new_n228,
    new_n229, new_n231, new_n232, new_n233, new_n234, new_n235, new_n236,
    new_n238, new_n239, new_n240, new_n241, new_n242, new_n243, new_n244,
    new_n245, new_n246, new_n247, new_n248, new_n250, new_n251, new_n252,
    new_n253, new_n254, new_n255, new_n256, new_n257, new_n258, new_n259,
    new_n261, new_n262, new_n263, new_n264, new_n265, new_n266, new_n267,
    new_n268, new_n270, new_n271, new_n272, new_n273, new_n274, new_n275,
    new_n276, new_n277, new_n278, new_n279, new_n280, new_n281, new_n282,
    new_n283, new_n284, new_n285, new_n286, new_n287, new_n288, new_n289,
    new_n290, new_n291, new_n292, new_n294, new_n295, new_n296, new_n297,
    new_n298, new_n299, new_n300, new_n301, new_n303, new_n304, new_n305,
    new_n306, new_n307, new_n308, new_n309, new_n312, new_n313, new_n314,
    new_n315, new_n316, new_n317, new_n318, new_n320, new_n321, new_n322,
    new_n323, new_n324, new_n325, new_n326, new_n329, new_n332, new_n333,
    new_n334, new_n336, new_n337, new_n338, new_n340;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor042aa1n02x5               g001(.a(\b[7] ), .b(\a[8] ), .o1(new_n97));
  nanp02aa1n02x5               g002(.a(\b[7] ), .b(\a[8] ), .o1(new_n98));
  nor042aa1n02x5               g003(.a(\b[6] ), .b(\a[7] ), .o1(new_n99));
  nanp02aa1n02x5               g004(.a(\b[6] ), .b(\a[7] ), .o1(new_n100));
  nona23aa1n02x4               g005(.a(new_n100), .b(new_n98), .c(new_n97), .d(new_n99), .out0(new_n101));
  xorc02aa1n02x5               g006(.a(\a[6] ), .b(\b[5] ), .out0(new_n102));
  xorc02aa1n02x5               g007(.a(\a[5] ), .b(\b[4] ), .out0(new_n103));
  nano22aa1n03x7               g008(.a(new_n101), .b(new_n103), .c(new_n102), .out0(new_n104));
  norp02aa1n02x5               g009(.a(\b[3] ), .b(\a[4] ), .o1(new_n105));
  nanp02aa1n02x5               g010(.a(\b[3] ), .b(\a[4] ), .o1(new_n106));
  norp02aa1n03x5               g011(.a(\b[2] ), .b(\a[3] ), .o1(new_n107));
  nanp02aa1n02x5               g012(.a(\b[2] ), .b(\a[3] ), .o1(new_n108));
  nona23aa1n06x5               g013(.a(new_n108), .b(new_n106), .c(new_n105), .d(new_n107), .out0(new_n109));
  nanp02aa1n02x5               g014(.a(\b[1] ), .b(\a[2] ), .o1(new_n110));
  nand02aa1n04x5               g015(.a(\b[0] ), .b(\a[1] ), .o1(new_n111));
  nor042aa1n02x5               g016(.a(\b[1] ), .b(\a[2] ), .o1(new_n112));
  oai012aa1n12x5               g017(.a(new_n110), .b(new_n112), .c(new_n111), .o1(new_n113));
  oaih12aa1n02x5               g018(.a(new_n106), .b(new_n107), .c(new_n105), .o1(new_n114));
  oai012aa1n12x5               g019(.a(new_n114), .b(new_n109), .c(new_n113), .o1(new_n115));
  aoi112aa1n02x5               g020(.a(\b[6] ), .b(\a[7] ), .c(\a[8] ), .d(\b[7] ), .o1(new_n116));
  nano23aa1n06x5               g021(.a(new_n97), .b(new_n99), .c(new_n100), .d(new_n98), .out0(new_n117));
  inv000aa1d42x5               g022(.a(\a[5] ), .o1(new_n118));
  inv000aa1d42x5               g023(.a(\b[4] ), .o1(new_n119));
  nanp02aa1n02x5               g024(.a(new_n119), .b(new_n118), .o1(new_n120));
  oaoi03aa1n02x5               g025(.a(\a[6] ), .b(\b[5] ), .c(new_n120), .o1(new_n121));
  nand42aa1n03x5               g026(.a(new_n117), .b(new_n121), .o1(new_n122));
  nona22aa1n03x5               g027(.a(new_n122), .b(new_n116), .c(new_n97), .out0(new_n123));
  aoi012aa1n02x5               g028(.a(new_n123), .b(new_n104), .c(new_n115), .o1(new_n124));
  oaoi03aa1n02x5               g029(.a(\a[9] ), .b(\b[8] ), .c(new_n124), .o1(new_n125));
  xorb03aa1n02x5               g030(.a(new_n125), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  nor002aa1d24x5               g031(.a(\b[9] ), .b(\a[10] ), .o1(new_n127));
  nand02aa1n08x5               g032(.a(\b[9] ), .b(\a[10] ), .o1(new_n128));
  nor002aa1n16x5               g033(.a(\b[8] ), .b(\a[9] ), .o1(new_n129));
  nanp02aa1n02x5               g034(.a(\b[8] ), .b(\a[9] ), .o1(new_n130));
  nano23aa1n06x5               g035(.a(new_n127), .b(new_n129), .c(new_n130), .d(new_n128), .out0(new_n131));
  aoai13aa1n02x5               g036(.a(new_n131), .b(new_n123), .c(new_n104), .d(new_n115), .o1(new_n132));
  oai012aa1d24x5               g037(.a(new_n128), .b(new_n129), .c(new_n127), .o1(new_n133));
  nor002aa1d32x5               g038(.a(\b[10] ), .b(\a[11] ), .o1(new_n134));
  nanp02aa1n04x5               g039(.a(\b[10] ), .b(\a[11] ), .o1(new_n135));
  norb02aa1n02x5               g040(.a(new_n135), .b(new_n134), .out0(new_n136));
  xnbna2aa1n03x5               g041(.a(new_n136), .b(new_n132), .c(new_n133), .out0(\s[11] ));
  inv000aa1d42x5               g042(.a(new_n134), .o1(new_n138));
  inv000aa1n03x5               g043(.a(new_n124), .o1(new_n139));
  inv000aa1d42x5               g044(.a(new_n133), .o1(new_n140));
  aoai13aa1n02x5               g045(.a(new_n136), .b(new_n140), .c(new_n139), .d(new_n131), .o1(new_n141));
  nor002aa1d32x5               g046(.a(\b[11] ), .b(\a[12] ), .o1(new_n142));
  nanp02aa1n04x5               g047(.a(\b[11] ), .b(\a[12] ), .o1(new_n143));
  norb02aa1n02x5               g048(.a(new_n143), .b(new_n142), .out0(new_n144));
  xnbna2aa1n03x5               g049(.a(new_n144), .b(new_n141), .c(new_n138), .out0(\s[12] ));
  nona23aa1n02x4               g050(.a(new_n130), .b(new_n128), .c(new_n127), .d(new_n129), .out0(new_n146));
  nona23aa1n09x5               g051(.a(new_n143), .b(new_n135), .c(new_n134), .d(new_n142), .out0(new_n147));
  norp02aa1n02x5               g052(.a(new_n147), .b(new_n146), .o1(new_n148));
  aoai13aa1n06x5               g053(.a(new_n148), .b(new_n123), .c(new_n104), .d(new_n115), .o1(new_n149));
  inv000aa1d42x5               g054(.a(new_n142), .o1(new_n150));
  nanp02aa1n02x5               g055(.a(new_n134), .b(new_n143), .o1(new_n151));
  oai112aa1n06x5               g056(.a(new_n151), .b(new_n150), .c(new_n147), .d(new_n133), .o1(new_n152));
  inv000aa1d42x5               g057(.a(new_n152), .o1(new_n153));
  nor022aa1n12x5               g058(.a(\b[12] ), .b(\a[13] ), .o1(new_n154));
  nanp02aa1n02x5               g059(.a(\b[12] ), .b(\a[13] ), .o1(new_n155));
  norb02aa1n02x5               g060(.a(new_n155), .b(new_n154), .out0(new_n156));
  xnbna2aa1n03x5               g061(.a(new_n156), .b(new_n149), .c(new_n153), .out0(\s[13] ));
  inv000aa1d42x5               g062(.a(new_n154), .o1(new_n158));
  aob012aa1n02x5               g063(.a(new_n156), .b(new_n149), .c(new_n153), .out0(new_n159));
  norp02aa1n02x5               g064(.a(\b[13] ), .b(\a[14] ), .o1(new_n160));
  nanp02aa1n02x5               g065(.a(\b[13] ), .b(\a[14] ), .o1(new_n161));
  norb02aa1n02x5               g066(.a(new_n161), .b(new_n160), .out0(new_n162));
  xnbna2aa1n03x5               g067(.a(new_n162), .b(new_n159), .c(new_n158), .out0(\s[14] ));
  nona23aa1n03x5               g068(.a(new_n161), .b(new_n155), .c(new_n154), .d(new_n160), .out0(new_n164));
  oai012aa1n02x5               g069(.a(new_n161), .b(new_n160), .c(new_n154), .o1(new_n165));
  aoai13aa1n03x5               g070(.a(new_n165), .b(new_n164), .c(new_n149), .d(new_n153), .o1(new_n166));
  xorb03aa1n02x5               g071(.a(new_n166), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  norp02aa1n06x5               g072(.a(\b[14] ), .b(\a[15] ), .o1(new_n168));
  nanp02aa1n02x5               g073(.a(\b[14] ), .b(\a[15] ), .o1(new_n169));
  norb02aa1n02x5               g074(.a(new_n169), .b(new_n168), .out0(new_n170));
  norp02aa1n02x5               g075(.a(\b[15] ), .b(\a[16] ), .o1(new_n171));
  nanp02aa1n02x5               g076(.a(\b[15] ), .b(\a[16] ), .o1(new_n172));
  norb02aa1n02x5               g077(.a(new_n172), .b(new_n171), .out0(new_n173));
  aoi112aa1n02x5               g078(.a(new_n173), .b(new_n168), .c(new_n166), .d(new_n170), .o1(new_n174));
  aoai13aa1n02x5               g079(.a(new_n173), .b(new_n168), .c(new_n166), .d(new_n169), .o1(new_n175));
  norb02aa1n03x4               g080(.a(new_n175), .b(new_n174), .out0(\s[16] ));
  nano23aa1n02x4               g081(.a(new_n134), .b(new_n142), .c(new_n143), .d(new_n135), .out0(new_n177));
  nona23aa1n12x5               g082(.a(new_n172), .b(new_n169), .c(new_n168), .d(new_n171), .out0(new_n178));
  nano23aa1n03x7               g083(.a(new_n178), .b(new_n164), .c(new_n177), .d(new_n131), .out0(new_n179));
  aoai13aa1n06x5               g084(.a(new_n179), .b(new_n123), .c(new_n104), .d(new_n115), .o1(new_n180));
  nor002aa1n03x5               g085(.a(new_n178), .b(new_n164), .o1(new_n181));
  aoi112aa1n02x5               g086(.a(\b[14] ), .b(\a[15] ), .c(\a[16] ), .d(\b[15] ), .o1(new_n182));
  oai022aa1n04x7               g087(.a(new_n178), .b(new_n165), .c(\b[15] ), .d(\a[16] ), .o1(new_n183));
  aoi112aa1n09x5               g088(.a(new_n183), .b(new_n182), .c(new_n152), .d(new_n181), .o1(new_n184));
  nand02aa1d06x5               g089(.a(new_n184), .b(new_n180), .o1(new_n185));
  xorb03aa1n02x5               g090(.a(new_n185), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv040aa1d32x5               g091(.a(\a[18] ), .o1(new_n187));
  inv000aa1d42x5               g092(.a(\a[17] ), .o1(new_n188));
  inv000aa1d42x5               g093(.a(\b[16] ), .o1(new_n189));
  oaoi03aa1n03x5               g094(.a(new_n188), .b(new_n189), .c(new_n185), .o1(new_n190));
  xorb03aa1n02x5               g095(.a(new_n190), .b(\b[17] ), .c(new_n187), .out0(\s[18] ));
  xroi22aa1d06x4               g096(.a(new_n188), .b(\b[16] ), .c(new_n187), .d(\b[17] ), .out0(new_n192));
  inv000aa1d42x5               g097(.a(new_n192), .o1(new_n193));
  oai022aa1n02x5               g098(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n194));
  oaib12aa1n02x5               g099(.a(new_n194), .b(new_n187), .c(\b[17] ), .out0(new_n195));
  aoai13aa1n06x5               g100(.a(new_n195), .b(new_n193), .c(new_n184), .d(new_n180), .o1(new_n196));
  xorb03aa1n03x5               g101(.a(new_n196), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g102(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor002aa1d32x5               g103(.a(\b[18] ), .b(\a[19] ), .o1(new_n199));
  nand02aa1n04x5               g104(.a(\b[18] ), .b(\a[19] ), .o1(new_n200));
  nanb02aa1n06x5               g105(.a(new_n199), .b(new_n200), .out0(new_n201));
  inv000aa1d42x5               g106(.a(new_n201), .o1(new_n202));
  inv000aa1d42x5               g107(.a(\b[19] ), .o1(new_n203));
  nanb02aa1n03x5               g108(.a(\a[20] ), .b(new_n203), .out0(new_n204));
  nanp02aa1n04x5               g109(.a(\b[19] ), .b(\a[20] ), .o1(new_n205));
  nanp02aa1n09x5               g110(.a(new_n204), .b(new_n205), .o1(new_n206));
  inv000aa1d42x5               g111(.a(new_n206), .o1(new_n207));
  aoi112aa1n02x7               g112(.a(new_n199), .b(new_n207), .c(new_n196), .d(new_n202), .o1(new_n208));
  inv000aa1d42x5               g113(.a(new_n199), .o1(new_n209));
  tech160nm_finand02aa1n03p5x5 g114(.a(new_n196), .b(new_n202), .o1(new_n210));
  aoi012aa1n03x5               g115(.a(new_n206), .b(new_n210), .c(new_n209), .o1(new_n211));
  norp02aa1n03x5               g116(.a(new_n211), .b(new_n208), .o1(\s[20] ));
  norp02aa1n02x5               g117(.a(\b[19] ), .b(\a[20] ), .o1(new_n213));
  nona23aa1n12x5               g118(.a(new_n205), .b(new_n200), .c(new_n199), .d(new_n213), .out0(new_n214));
  inv000aa1d42x5               g119(.a(new_n214), .o1(new_n215));
  nanp02aa1n02x5               g120(.a(new_n192), .b(new_n215), .o1(new_n216));
  nanp02aa1n02x5               g121(.a(new_n199), .b(new_n205), .o1(new_n217));
  nor003aa1n02x5               g122(.a(new_n195), .b(new_n201), .c(new_n206), .o1(new_n218));
  nano22aa1n02x4               g123(.a(new_n218), .b(new_n204), .c(new_n217), .out0(new_n219));
  aoai13aa1n06x5               g124(.a(new_n219), .b(new_n216), .c(new_n184), .d(new_n180), .o1(new_n220));
  xorb03aa1n03x5               g125(.a(new_n220), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor042aa1n03x5               g126(.a(\b[20] ), .b(\a[21] ), .o1(new_n222));
  xorc02aa1n12x5               g127(.a(\a[21] ), .b(\b[20] ), .out0(new_n223));
  xorc02aa1n12x5               g128(.a(\a[22] ), .b(\b[21] ), .out0(new_n224));
  aoi112aa1n02x7               g129(.a(new_n222), .b(new_n224), .c(new_n220), .d(new_n223), .o1(new_n225));
  inv000aa1d42x5               g130(.a(new_n222), .o1(new_n226));
  tech160nm_finand02aa1n03p5x5 g131(.a(new_n220), .b(new_n223), .o1(new_n227));
  inv000aa1d42x5               g132(.a(new_n224), .o1(new_n228));
  aoi012aa1n03x5               g133(.a(new_n228), .b(new_n227), .c(new_n226), .o1(new_n229));
  norp02aa1n03x5               g134(.a(new_n229), .b(new_n225), .o1(\s[22] ));
  oai112aa1n03x5               g135(.a(new_n217), .b(new_n204), .c(new_n214), .d(new_n195), .o1(new_n231));
  and002aa1n02x5               g136(.a(new_n224), .b(new_n223), .o(new_n232));
  oaoi03aa1n02x5               g137(.a(\a[22] ), .b(\b[21] ), .c(new_n226), .o1(new_n233));
  aoi012aa1n02x5               g138(.a(new_n233), .b(new_n231), .c(new_n232), .o1(new_n234));
  nona23aa1d18x5               g139(.a(new_n223), .b(new_n192), .c(new_n228), .d(new_n214), .out0(new_n235));
  aoai13aa1n06x5               g140(.a(new_n234), .b(new_n235), .c(new_n184), .d(new_n180), .o1(new_n236));
  xorb03aa1n02x5               g141(.a(new_n236), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  nor042aa1n04x5               g142(.a(\b[22] ), .b(\a[23] ), .o1(new_n238));
  nand42aa1n02x5               g143(.a(\b[22] ), .b(\a[23] ), .o1(new_n239));
  norb02aa1n02x5               g144(.a(new_n239), .b(new_n238), .out0(new_n240));
  nor042aa1n02x5               g145(.a(\b[23] ), .b(\a[24] ), .o1(new_n241));
  nand42aa1n02x5               g146(.a(\b[23] ), .b(\a[24] ), .o1(new_n242));
  norb02aa1n02x5               g147(.a(new_n242), .b(new_n241), .out0(new_n243));
  aoi112aa1n03x4               g148(.a(new_n238), .b(new_n243), .c(new_n236), .d(new_n240), .o1(new_n244));
  inv000aa1d42x5               g149(.a(new_n238), .o1(new_n245));
  nand42aa1n02x5               g150(.a(new_n236), .b(new_n240), .o1(new_n246));
  inv000aa1d42x5               g151(.a(new_n243), .o1(new_n247));
  aoi012aa1n03x5               g152(.a(new_n247), .b(new_n246), .c(new_n245), .o1(new_n248));
  norp02aa1n03x5               g153(.a(new_n248), .b(new_n244), .o1(\s[24] ));
  nano23aa1n09x5               g154(.a(new_n238), .b(new_n241), .c(new_n242), .d(new_n239), .out0(new_n250));
  inv000aa1d42x5               g155(.a(new_n250), .o1(new_n251));
  nona23aa1n03x5               g156(.a(new_n232), .b(new_n192), .c(new_n251), .d(new_n214), .out0(new_n252));
  aoi112aa1n02x5               g157(.a(\b[22] ), .b(\a[23] ), .c(\a[24] ), .d(\b[23] ), .o1(new_n253));
  nanp02aa1n02x5               g158(.a(new_n250), .b(new_n233), .o1(new_n254));
  nona22aa1n02x4               g159(.a(new_n254), .b(new_n253), .c(new_n241), .out0(new_n255));
  nand23aa1n03x5               g160(.a(new_n250), .b(new_n223), .c(new_n224), .o1(new_n256));
  inv040aa1n03x5               g161(.a(new_n256), .o1(new_n257));
  aoi012aa1n02x5               g162(.a(new_n255), .b(new_n231), .c(new_n257), .o1(new_n258));
  aoai13aa1n06x5               g163(.a(new_n258), .b(new_n252), .c(new_n184), .d(new_n180), .o1(new_n259));
  xorb03aa1n02x5               g164(.a(new_n259), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g165(.a(\b[24] ), .b(\a[25] ), .o1(new_n261));
  xorc02aa1n02x5               g166(.a(\a[25] ), .b(\b[24] ), .out0(new_n262));
  xorc02aa1n12x5               g167(.a(\a[26] ), .b(\b[25] ), .out0(new_n263));
  aoi112aa1n03x4               g168(.a(new_n261), .b(new_n263), .c(new_n259), .d(new_n262), .o1(new_n264));
  inv000aa1d42x5               g169(.a(new_n261), .o1(new_n265));
  tech160nm_finand02aa1n05x5   g170(.a(new_n259), .b(new_n262), .o1(new_n266));
  inv000aa1d42x5               g171(.a(new_n263), .o1(new_n267));
  aoi012aa1n03x5               g172(.a(new_n267), .b(new_n266), .c(new_n265), .o1(new_n268));
  nor002aa1n02x5               g173(.a(new_n268), .b(new_n264), .o1(\s[26] ));
  nanp03aa1n02x5               g174(.a(new_n117), .b(new_n102), .c(new_n103), .o1(new_n270));
  nano23aa1n02x4               g175(.a(new_n105), .b(new_n107), .c(new_n108), .d(new_n106), .out0(new_n271));
  inv000aa1d42x5               g176(.a(new_n113), .o1(new_n272));
  aobi12aa1n02x5               g177(.a(new_n114), .b(new_n271), .c(new_n272), .out0(new_n273));
  aoi112aa1n02x5               g178(.a(new_n116), .b(new_n97), .c(new_n117), .d(new_n121), .o1(new_n274));
  nanp02aa1n02x5               g179(.a(new_n181), .b(new_n148), .o1(new_n275));
  oaoi13aa1n04x5               g180(.a(new_n275), .b(new_n274), .c(new_n273), .d(new_n270), .o1(new_n276));
  nand42aa1n03x5               g181(.a(new_n152), .b(new_n181), .o1(new_n277));
  nona22aa1n02x5               g182(.a(new_n277), .b(new_n183), .c(new_n182), .out0(new_n278));
  and002aa1n02x7               g183(.a(new_n263), .b(new_n262), .o(new_n279));
  nano22aa1n12x5               g184(.a(new_n235), .b(new_n250), .c(new_n279), .out0(new_n280));
  oai012aa1n12x5               g185(.a(new_n280), .b(new_n278), .c(new_n276), .o1(new_n281));
  norp02aa1n02x5               g186(.a(\b[25] ), .b(\a[26] ), .o1(new_n282));
  inv000aa1d42x5               g187(.a(new_n282), .o1(new_n283));
  aoi112aa1n02x5               g188(.a(\b[24] ), .b(\a[25] ), .c(\a[26] ), .d(\b[25] ), .o1(new_n284));
  inv000aa1n02x5               g189(.a(new_n284), .o1(new_n285));
  aoi112aa1n02x5               g190(.a(new_n253), .b(new_n241), .c(new_n250), .d(new_n233), .o1(new_n286));
  inv000aa1n02x5               g191(.a(new_n279), .o1(new_n287));
  oaoi13aa1n03x5               g192(.a(new_n287), .b(new_n286), .c(new_n219), .d(new_n256), .o1(new_n288));
  nano22aa1n03x7               g193(.a(new_n288), .b(new_n283), .c(new_n285), .out0(new_n289));
  nor042aa1n03x5               g194(.a(\b[26] ), .b(\a[27] ), .o1(new_n290));
  nanp02aa1n02x5               g195(.a(\b[26] ), .b(\a[27] ), .o1(new_n291));
  norb02aa1n02x5               g196(.a(new_n291), .b(new_n290), .out0(new_n292));
  xnbna2aa1n03x5               g197(.a(new_n292), .b(new_n281), .c(new_n289), .out0(\s[27] ));
  xorc02aa1n02x5               g198(.a(\a[28] ), .b(\b[27] ), .out0(new_n294));
  aoai13aa1n06x5               g199(.a(new_n279), .b(new_n255), .c(new_n231), .d(new_n257), .o1(new_n295));
  nona22aa1n06x5               g200(.a(new_n295), .b(new_n284), .c(new_n282), .out0(new_n296));
  aoi112aa1n03x5               g201(.a(new_n296), .b(new_n290), .c(new_n185), .d(new_n280), .o1(new_n297));
  nano22aa1n03x5               g202(.a(new_n297), .b(new_n291), .c(new_n294), .out0(new_n298));
  inv000aa1d42x5               g203(.a(new_n290), .o1(new_n299));
  nanp03aa1n03x5               g204(.a(new_n281), .b(new_n289), .c(new_n299), .o1(new_n300));
  aoi012aa1n03x5               g205(.a(new_n294), .b(new_n300), .c(new_n291), .o1(new_n301));
  nor002aa1n02x5               g206(.a(new_n301), .b(new_n298), .o1(\s[28] ));
  and002aa1n02x5               g207(.a(new_n294), .b(new_n292), .o(new_n303));
  aoai13aa1n03x5               g208(.a(new_n303), .b(new_n296), .c(new_n185), .d(new_n280), .o1(new_n304));
  oao003aa1n02x5               g209(.a(\a[28] ), .b(\b[27] ), .c(new_n299), .carry(new_n305));
  xnrc02aa1n02x5               g210(.a(\b[28] ), .b(\a[29] ), .out0(new_n306));
  aoi012aa1n03x5               g211(.a(new_n306), .b(new_n304), .c(new_n305), .o1(new_n307));
  aobi12aa1n06x5               g212(.a(new_n303), .b(new_n281), .c(new_n289), .out0(new_n308));
  nano22aa1n03x5               g213(.a(new_n308), .b(new_n305), .c(new_n306), .out0(new_n309));
  norp02aa1n03x5               g214(.a(new_n307), .b(new_n309), .o1(\s[29] ));
  xorb03aa1n02x5               g215(.a(new_n111), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n02x4               g216(.a(new_n306), .b(new_n294), .c(new_n292), .out0(new_n312));
  aoai13aa1n03x5               g217(.a(new_n312), .b(new_n296), .c(new_n185), .d(new_n280), .o1(new_n313));
  oao003aa1n02x5               g218(.a(\a[29] ), .b(\b[28] ), .c(new_n305), .carry(new_n314));
  xnrc02aa1n02x5               g219(.a(\b[29] ), .b(\a[30] ), .out0(new_n315));
  aoi012aa1n03x5               g220(.a(new_n315), .b(new_n313), .c(new_n314), .o1(new_n316));
  aobi12aa1n02x7               g221(.a(new_n312), .b(new_n281), .c(new_n289), .out0(new_n317));
  nano22aa1n03x5               g222(.a(new_n317), .b(new_n314), .c(new_n315), .out0(new_n318));
  norp02aa1n03x5               g223(.a(new_n316), .b(new_n318), .o1(\s[30] ));
  nano23aa1n02x4               g224(.a(new_n315), .b(new_n306), .c(new_n294), .d(new_n292), .out0(new_n320));
  aobi12aa1n02x7               g225(.a(new_n320), .b(new_n281), .c(new_n289), .out0(new_n321));
  oao003aa1n02x5               g226(.a(\a[30] ), .b(\b[29] ), .c(new_n314), .carry(new_n322));
  xnrc02aa1n02x5               g227(.a(\b[30] ), .b(\a[31] ), .out0(new_n323));
  nano22aa1n03x5               g228(.a(new_n321), .b(new_n322), .c(new_n323), .out0(new_n324));
  aoai13aa1n03x5               g229(.a(new_n320), .b(new_n296), .c(new_n185), .d(new_n280), .o1(new_n325));
  aoi012aa1n02x7               g230(.a(new_n323), .b(new_n325), .c(new_n322), .o1(new_n326));
  nor002aa1n02x5               g231(.a(new_n326), .b(new_n324), .o1(\s[31] ));
  xnrb03aa1n02x5               g232(.a(new_n113), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g233(.a(\a[3] ), .b(\b[2] ), .c(new_n113), .o1(new_n329));
  xorb03aa1n02x5               g234(.a(new_n329), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g235(.a(new_n115), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oai112aa1n03x5               g236(.a(new_n103), .b(new_n114), .c(new_n109), .d(new_n113), .o1(new_n332));
  oai112aa1n02x5               g237(.a(new_n332), .b(new_n102), .c(new_n119), .d(new_n118), .o1(new_n333));
  oaoi13aa1n02x5               g238(.a(new_n102), .b(new_n332), .c(new_n118), .d(new_n119), .o1(new_n334));
  norb02aa1n02x5               g239(.a(new_n333), .b(new_n334), .out0(\s[6] ));
  nanb02aa1n02x5               g240(.a(new_n99), .b(new_n100), .out0(new_n336));
  oaoi13aa1n04x5               g241(.a(new_n336), .b(new_n333), .c(\a[6] ), .d(\b[5] ), .o1(new_n337));
  oai112aa1n02x5               g242(.a(new_n333), .b(new_n336), .c(\b[5] ), .d(\a[6] ), .o1(new_n338));
  norb02aa1n02x5               g243(.a(new_n338), .b(new_n337), .out0(\s[7] ));
  norp02aa1n02x5               g244(.a(new_n337), .b(new_n99), .o1(new_n340));
  xnrb03aa1n03x5               g245(.a(new_n340), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xnrb03aa1n02x5               g246(.a(new_n124), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


