// Benchmark "adder" written by ABC on Wed Jul 17 17:49:50 2024

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
    new_n125, new_n126, new_n127, new_n128, new_n129, new_n131, new_n132,
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n138, new_n139,
    new_n140, new_n141, new_n142, new_n144, new_n145, new_n146, new_n147,
    new_n148, new_n149, new_n150, new_n151, new_n152, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n161, new_n162,
    new_n163, new_n164, new_n165, new_n167, new_n168, new_n169, new_n170,
    new_n171, new_n172, new_n174, new_n175, new_n176, new_n177, new_n178,
    new_n179, new_n180, new_n181, new_n183, new_n184, new_n185, new_n186,
    new_n187, new_n188, new_n189, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n196, new_n197, new_n198, new_n199, new_n200, new_n201,
    new_n202, new_n203, new_n205, new_n206, new_n207, new_n208, new_n209,
    new_n210, new_n211, new_n213, new_n214, new_n215, new_n216, new_n217,
    new_n218, new_n219, new_n222, new_n223, new_n224, new_n225, new_n226,
    new_n227, new_n228, new_n229, new_n230, new_n231, new_n232, new_n234,
    new_n235, new_n236, new_n237, new_n238, new_n239, new_n240, new_n241,
    new_n243, new_n244, new_n245, new_n246, new_n247, new_n249, new_n250,
    new_n251, new_n252, new_n253, new_n254, new_n255, new_n256, new_n257,
    new_n259, new_n260, new_n261, new_n262, new_n263, new_n265, new_n266,
    new_n267, new_n268, new_n269, new_n270, new_n271, new_n272, new_n273,
    new_n274, new_n275, new_n276, new_n277, new_n279, new_n280, new_n281,
    new_n282, new_n284, new_n285, new_n286, new_n287, new_n288, new_n289,
    new_n290, new_n291, new_n292, new_n293, new_n294, new_n295, new_n297,
    new_n298, new_n299, new_n300, new_n301, new_n302, new_n303, new_n304,
    new_n305, new_n306, new_n307, new_n308, new_n309, new_n311, new_n312,
    new_n313, new_n314, new_n315, new_n316, new_n317, new_n318, new_n319,
    new_n322, new_n323, new_n324, new_n325, new_n326, new_n327, new_n328,
    new_n330, new_n331, new_n332, new_n333, new_n334, new_n335, new_n336,
    new_n337, new_n338, new_n341, new_n342, new_n343, new_n346, new_n348,
    new_n349, new_n350, new_n352, new_n353;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor002aa1d32x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(new_n97), .o1(new_n98));
  orn002aa1n02x7               g003(.a(\a[2] ), .b(\b[1] ), .o(new_n99));
  nand02aa1n04x5               g004(.a(\b[0] ), .b(\a[1] ), .o1(new_n100));
  aob012aa1n12x5               g005(.a(new_n100), .b(\b[1] ), .c(\a[2] ), .out0(new_n101));
  nor002aa1n16x5               g006(.a(\b[2] ), .b(\a[3] ), .o1(new_n102));
  nanp02aa1n02x5               g007(.a(\b[2] ), .b(\a[3] ), .o1(new_n103));
  nanb02aa1n02x5               g008(.a(new_n102), .b(new_n103), .out0(new_n104));
  nor022aa1n08x5               g009(.a(\b[3] ), .b(\a[4] ), .o1(new_n105));
  nor042aa1n09x5               g010(.a(new_n105), .b(new_n102), .o1(new_n106));
  aoai13aa1n06x5               g011(.a(new_n106), .b(new_n104), .c(new_n101), .d(new_n99), .o1(new_n107));
  nanp02aa1n02x5               g012(.a(\b[4] ), .b(\a[5] ), .o1(new_n108));
  norp02aa1n09x5               g013(.a(\b[4] ), .b(\a[5] ), .o1(new_n109));
  aoi012aa1n06x5               g014(.a(new_n109), .b(\a[6] ), .c(\b[5] ), .o1(new_n110));
  inv000aa1d42x5               g015(.a(\a[8] ), .o1(new_n111));
  inv000aa1d42x5               g016(.a(\b[7] ), .o1(new_n112));
  aoi022aa1d24x5               g017(.a(new_n112), .b(new_n111), .c(\a[4] ), .d(\b[3] ), .o1(new_n113));
  nanp02aa1n02x5               g018(.a(\b[7] ), .b(\a[8] ), .o1(new_n114));
  nor002aa1d32x5               g019(.a(\b[6] ), .b(\a[7] ), .o1(new_n115));
  nand42aa1n02x5               g020(.a(\b[6] ), .b(\a[7] ), .o1(new_n116));
  nor002aa1d32x5               g021(.a(\b[5] ), .b(\a[6] ), .o1(new_n117));
  nona23aa1n03x5               g022(.a(new_n116), .b(new_n114), .c(new_n117), .d(new_n115), .out0(new_n118));
  nano32aa1n02x5               g023(.a(new_n118), .b(new_n113), .c(new_n110), .d(new_n108), .out0(new_n119));
  nanp02aa1n04x5               g024(.a(\b[5] ), .b(\a[6] ), .o1(new_n120));
  oaoi13aa1n02x5               g025(.a(new_n115), .b(new_n120), .c(new_n117), .d(new_n109), .o1(new_n121));
  aoi022aa1n02x5               g026(.a(\b[7] ), .b(\a[8] ), .c(\a[7] ), .d(\b[6] ), .o1(new_n122));
  obai22aa1n03x5               g027(.a(new_n122), .b(new_n121), .c(\a[8] ), .d(\b[7] ), .out0(new_n123));
  nanp02aa1n02x5               g028(.a(\b[8] ), .b(\a[9] ), .o1(new_n124));
  norb02aa1n02x5               g029(.a(new_n124), .b(new_n97), .out0(new_n125));
  aoai13aa1n02x5               g030(.a(new_n125), .b(new_n123), .c(new_n119), .d(new_n107), .o1(new_n126));
  nor002aa1n16x5               g031(.a(\b[9] ), .b(\a[10] ), .o1(new_n127));
  nand02aa1n04x5               g032(.a(\b[9] ), .b(\a[10] ), .o1(new_n128));
  norb02aa1n02x5               g033(.a(new_n128), .b(new_n127), .out0(new_n129));
  xnbna2aa1n03x5               g034(.a(new_n129), .b(new_n126), .c(new_n98), .out0(\s[10] ));
  nanp02aa1n02x5               g035(.a(new_n101), .b(new_n99), .o1(new_n131));
  norb02aa1n03x5               g036(.a(new_n103), .b(new_n102), .out0(new_n132));
  inv000aa1d42x5               g037(.a(new_n106), .o1(new_n133));
  aoi012aa1n02x5               g038(.a(new_n133), .b(new_n131), .c(new_n132), .o1(new_n134));
  nano22aa1n02x4               g039(.a(new_n109), .b(new_n108), .c(new_n120), .out0(new_n135));
  nano23aa1n02x4               g040(.a(new_n117), .b(new_n115), .c(new_n114), .d(new_n116), .out0(new_n136));
  nano32aa1n03x7               g041(.a(new_n134), .b(new_n136), .c(new_n135), .d(new_n113), .out0(new_n137));
  nona23aa1n03x5               g042(.a(new_n128), .b(new_n124), .c(new_n97), .d(new_n127), .out0(new_n138));
  oabi12aa1n02x5               g043(.a(new_n138), .b(new_n137), .c(new_n123), .out0(new_n139));
  oai012aa1n02x5               g044(.a(new_n128), .b(new_n127), .c(new_n97), .o1(new_n140));
  xnrc02aa1n12x5               g045(.a(\b[10] ), .b(\a[11] ), .out0(new_n141));
  inv000aa1d42x5               g046(.a(new_n141), .o1(new_n142));
  xnbna2aa1n03x5               g047(.a(new_n142), .b(new_n139), .c(new_n140), .out0(\s[11] ));
  inv040aa1n09x5               g048(.a(\a[11] ), .o1(new_n144));
  inv000aa1d42x5               g049(.a(\b[10] ), .o1(new_n145));
  tech160nm_finand02aa1n05x5   g050(.a(new_n145), .b(new_n144), .o1(new_n146));
  aob012aa1n02x5               g051(.a(new_n142), .b(new_n139), .c(new_n140), .out0(new_n147));
  inv040aa1d32x5               g052(.a(\a[12] ), .o1(new_n148));
  inv040aa1n18x5               g053(.a(\b[11] ), .o1(new_n149));
  nand02aa1n04x5               g054(.a(new_n149), .b(new_n148), .o1(new_n150));
  nanp02aa1n04x5               g055(.a(\b[11] ), .b(\a[12] ), .o1(new_n151));
  nanp02aa1n02x5               g056(.a(new_n150), .b(new_n151), .o1(new_n152));
  xobna2aa1n03x5               g057(.a(new_n152), .b(new_n147), .c(new_n146), .out0(\s[12] ));
  nor003aa1n02x5               g058(.a(new_n138), .b(new_n141), .c(new_n152), .o1(new_n154));
  aoai13aa1n02x5               g059(.a(new_n154), .b(new_n123), .c(new_n119), .d(new_n107), .o1(new_n155));
  oai112aa1n06x5               g060(.a(new_n150), .b(new_n151), .c(new_n145), .d(new_n144), .o1(new_n156));
  oai112aa1n06x5               g061(.a(new_n128), .b(new_n146), .c(new_n127), .d(new_n97), .o1(new_n157));
  aoi112aa1n03x5               g062(.a(\b[10] ), .b(\a[11] ), .c(\a[12] ), .d(\b[11] ), .o1(new_n158));
  norb02aa1n09x5               g063(.a(new_n150), .b(new_n158), .out0(new_n159));
  oai012aa1d24x5               g064(.a(new_n159), .b(new_n157), .c(new_n156), .o1(new_n160));
  inv000aa1d42x5               g065(.a(new_n160), .o1(new_n161));
  nor002aa1d32x5               g066(.a(\b[12] ), .b(\a[13] ), .o1(new_n162));
  nand42aa1n08x5               g067(.a(\b[12] ), .b(\a[13] ), .o1(new_n163));
  nanb02aa1n12x5               g068(.a(new_n162), .b(new_n163), .out0(new_n164));
  inv000aa1d42x5               g069(.a(new_n164), .o1(new_n165));
  xnbna2aa1n03x5               g070(.a(new_n165), .b(new_n155), .c(new_n161), .out0(\s[13] ));
  inv000aa1d42x5               g071(.a(new_n162), .o1(new_n167));
  nanp02aa1n06x5               g072(.a(new_n155), .b(new_n161), .o1(new_n168));
  nanp02aa1n02x5               g073(.a(new_n168), .b(new_n165), .o1(new_n169));
  nor022aa1n06x5               g074(.a(\b[13] ), .b(\a[14] ), .o1(new_n170));
  nand02aa1d08x5               g075(.a(\b[13] ), .b(\a[14] ), .o1(new_n171));
  nanb02aa1n02x5               g076(.a(new_n170), .b(new_n171), .out0(new_n172));
  xobna2aa1n03x5               g077(.a(new_n172), .b(new_n169), .c(new_n167), .out0(\s[14] ));
  nona23aa1d24x5               g078(.a(new_n171), .b(new_n163), .c(new_n162), .d(new_n170), .out0(new_n174));
  inv000aa1d42x5               g079(.a(new_n174), .o1(new_n175));
  oaoi03aa1n02x5               g080(.a(\a[14] ), .b(\b[13] ), .c(new_n167), .o1(new_n176));
  nor042aa1n12x5               g081(.a(\b[14] ), .b(\a[15] ), .o1(new_n177));
  nanp02aa1n02x5               g082(.a(\b[14] ), .b(\a[15] ), .o1(new_n178));
  norb02aa1n06x5               g083(.a(new_n178), .b(new_n177), .out0(new_n179));
  aoai13aa1n06x5               g084(.a(new_n179), .b(new_n176), .c(new_n168), .d(new_n175), .o1(new_n180));
  aoi112aa1n02x5               g085(.a(new_n179), .b(new_n176), .c(new_n168), .d(new_n175), .o1(new_n181));
  norb02aa1n02x5               g086(.a(new_n180), .b(new_n181), .out0(\s[15] ));
  nor042aa1n12x5               g087(.a(\b[15] ), .b(\a[16] ), .o1(new_n183));
  nand02aa1d24x5               g088(.a(\b[15] ), .b(\a[16] ), .o1(new_n184));
  norb02aa1n12x5               g089(.a(new_n184), .b(new_n183), .out0(new_n185));
  inv000aa1d42x5               g090(.a(new_n183), .o1(new_n186));
  aoi012aa1n02x5               g091(.a(new_n177), .b(new_n186), .c(new_n184), .o1(new_n187));
  inv000aa1d42x5               g092(.a(new_n177), .o1(new_n188));
  nand22aa1n03x5               g093(.a(new_n180), .b(new_n188), .o1(new_n189));
  aoi022aa1n02x5               g094(.a(new_n189), .b(new_n185), .c(new_n180), .d(new_n187), .o1(\s[16] ));
  nand02aa1n03x5               g095(.a(new_n119), .b(new_n107), .o1(new_n191));
  inv000aa1d42x5               g096(.a(new_n115), .o1(new_n192));
  tech160nm_fioai012aa1n05x5   g097(.a(new_n120), .b(new_n117), .c(new_n109), .o1(new_n193));
  nanp02aa1n02x5               g098(.a(new_n193), .b(new_n192), .o1(new_n194));
  aoi022aa1n09x5               g099(.a(new_n194), .b(new_n122), .c(new_n112), .d(new_n111), .o1(new_n195));
  nano22aa1d15x5               g100(.a(new_n174), .b(new_n179), .c(new_n185), .out0(new_n196));
  nona32aa1n03x5               g101(.a(new_n196), .b(new_n138), .c(new_n152), .d(new_n141), .out0(new_n197));
  inv000aa1d42x5               g102(.a(new_n184), .o1(new_n198));
  oai022aa1n02x5               g103(.a(\a[13] ), .b(\b[12] ), .c(\b[13] ), .d(\a[14] ), .o1(new_n199));
  nanp03aa1n02x5               g104(.a(new_n199), .b(new_n171), .c(new_n178), .o1(new_n200));
  aoai13aa1n12x5               g105(.a(new_n186), .b(new_n198), .c(new_n200), .d(new_n188), .o1(new_n201));
  aoi012aa1d18x5               g106(.a(new_n201), .b(new_n160), .c(new_n196), .o1(new_n202));
  aoai13aa1n12x5               g107(.a(new_n202), .b(new_n197), .c(new_n191), .d(new_n195), .o1(new_n203));
  xorb03aa1n02x5               g108(.a(new_n203), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv000aa1d42x5               g109(.a(\a[17] ), .o1(new_n205));
  inv000aa1d48x5               g110(.a(\b[16] ), .o1(new_n206));
  nand42aa1n03x5               g111(.a(new_n206), .b(new_n205), .o1(new_n207));
  oaib12aa1n06x5               g112(.a(new_n203), .b(new_n206), .c(\a[17] ), .out0(new_n208));
  nor002aa1d32x5               g113(.a(\b[17] ), .b(\a[18] ), .o1(new_n209));
  nand02aa1d20x5               g114(.a(\b[17] ), .b(\a[18] ), .o1(new_n210));
  norb02aa1n02x5               g115(.a(new_n210), .b(new_n209), .out0(new_n211));
  xnbna2aa1n03x5               g116(.a(new_n211), .b(new_n208), .c(new_n207), .out0(\s[18] ));
  nand42aa1n03x5               g117(.a(\b[16] ), .b(\a[17] ), .o1(new_n213));
  nano32aa1n03x7               g118(.a(new_n209), .b(new_n207), .c(new_n210), .d(new_n213), .out0(new_n214));
  aoai13aa1n12x5               g119(.a(new_n210), .b(new_n209), .c(new_n205), .d(new_n206), .o1(new_n215));
  inv000aa1d42x5               g120(.a(new_n215), .o1(new_n216));
  tech160nm_fixorc02aa1n04x5   g121(.a(\a[19] ), .b(\b[18] ), .out0(new_n217));
  aoai13aa1n06x5               g122(.a(new_n217), .b(new_n216), .c(new_n203), .d(new_n214), .o1(new_n218));
  aoi112aa1n02x5               g123(.a(new_n217), .b(new_n216), .c(new_n203), .d(new_n214), .o1(new_n219));
  norb02aa1n02x5               g124(.a(new_n218), .b(new_n219), .out0(\s[19] ));
  xnrc02aa1n02x5               g125(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  xorc02aa1n12x5               g126(.a(\a[20] ), .b(\b[19] ), .out0(new_n222));
  inv000aa1d42x5               g127(.a(\a[19] ), .o1(new_n223));
  inv000aa1d42x5               g128(.a(\b[18] ), .o1(new_n224));
  inv000aa1d42x5               g129(.a(\a[20] ), .o1(new_n225));
  inv000aa1d42x5               g130(.a(\b[19] ), .o1(new_n226));
  nanp02aa1n02x5               g131(.a(new_n226), .b(new_n225), .o1(new_n227));
  nanp02aa1n02x5               g132(.a(\b[19] ), .b(\a[20] ), .o1(new_n228));
  aoi022aa1n02x5               g133(.a(new_n227), .b(new_n228), .c(new_n224), .d(new_n223), .o1(new_n229));
  nor042aa1n06x5               g134(.a(\b[18] ), .b(\a[19] ), .o1(new_n230));
  inv000aa1d42x5               g135(.a(new_n230), .o1(new_n231));
  nanp02aa1n03x5               g136(.a(new_n218), .b(new_n231), .o1(new_n232));
  aoi022aa1n02x7               g137(.a(new_n232), .b(new_n222), .c(new_n218), .d(new_n229), .o1(\s[20] ));
  nand23aa1n04x5               g138(.a(new_n214), .b(new_n217), .c(new_n222), .o1(new_n234));
  inv000aa1d42x5               g139(.a(new_n234), .o1(new_n235));
  oai112aa1n02x5               g140(.a(new_n227), .b(new_n228), .c(new_n224), .d(new_n223), .o1(new_n236));
  tech160nm_fioaoi03aa1n03p5x5 g141(.a(new_n225), .b(new_n226), .c(new_n230), .o1(new_n237));
  oai013aa1n03x5               g142(.a(new_n237), .b(new_n236), .c(new_n215), .d(new_n230), .o1(new_n238));
  xorc02aa1n12x5               g143(.a(\a[21] ), .b(\b[20] ), .out0(new_n239));
  aoai13aa1n06x5               g144(.a(new_n239), .b(new_n238), .c(new_n203), .d(new_n235), .o1(new_n240));
  aoi112aa1n02x5               g145(.a(new_n239), .b(new_n238), .c(new_n203), .d(new_n235), .o1(new_n241));
  norb02aa1n02x5               g146(.a(new_n240), .b(new_n241), .out0(\s[21] ));
  xorc02aa1n12x5               g147(.a(\a[22] ), .b(\b[21] ), .out0(new_n243));
  norp02aa1n02x5               g148(.a(\b[20] ), .b(\a[21] ), .o1(new_n244));
  norp02aa1n02x5               g149(.a(new_n243), .b(new_n244), .o1(new_n245));
  inv000aa1n03x5               g150(.a(new_n244), .o1(new_n246));
  nand22aa1n03x5               g151(.a(new_n240), .b(new_n246), .o1(new_n247));
  aoi022aa1n02x7               g152(.a(new_n247), .b(new_n243), .c(new_n240), .d(new_n245), .o1(\s[22] ));
  nand02aa1d06x5               g153(.a(new_n243), .b(new_n239), .o1(new_n249));
  nano32aa1n03x7               g154(.a(new_n249), .b(new_n214), .c(new_n217), .d(new_n222), .out0(new_n250));
  nona22aa1n09x5               g155(.a(new_n231), .b(new_n236), .c(new_n215), .out0(new_n251));
  oaoi03aa1n02x5               g156(.a(\a[22] ), .b(\b[21] ), .c(new_n246), .o1(new_n252));
  inv000aa1n02x5               g157(.a(new_n252), .o1(new_n253));
  aoai13aa1n12x5               g158(.a(new_n253), .b(new_n249), .c(new_n251), .d(new_n237), .o1(new_n254));
  xorc02aa1n02x5               g159(.a(\a[23] ), .b(\b[22] ), .out0(new_n255));
  aoai13aa1n06x5               g160(.a(new_n255), .b(new_n254), .c(new_n203), .d(new_n250), .o1(new_n256));
  aoi112aa1n02x5               g161(.a(new_n255), .b(new_n254), .c(new_n203), .d(new_n250), .o1(new_n257));
  norb02aa1n02x5               g162(.a(new_n256), .b(new_n257), .out0(\s[23] ));
  xorc02aa1n02x5               g163(.a(\a[24] ), .b(\b[23] ), .out0(new_n259));
  norp02aa1n02x5               g164(.a(\b[22] ), .b(\a[23] ), .o1(new_n260));
  norp02aa1n02x5               g165(.a(new_n259), .b(new_n260), .o1(new_n261));
  inv000aa1d42x5               g166(.a(\a[23] ), .o1(new_n262));
  oaib12aa1n06x5               g167(.a(new_n256), .b(\b[22] ), .c(new_n262), .out0(new_n263));
  aoi022aa1n02x7               g168(.a(new_n263), .b(new_n259), .c(new_n256), .d(new_n261), .o1(\s[24] ));
  inv000aa1d42x5               g169(.a(\a[24] ), .o1(new_n265));
  xroi22aa1d04x5               g170(.a(new_n262), .b(\b[22] ), .c(new_n265), .d(\b[23] ), .out0(new_n266));
  nano32aa1n02x5               g171(.a(new_n234), .b(new_n266), .c(new_n239), .d(new_n243), .out0(new_n267));
  nand02aa1d06x5               g172(.a(new_n203), .b(new_n267), .o1(new_n268));
  inv000aa1d42x5               g173(.a(\b[23] ), .o1(new_n269));
  tech160nm_fioaoi03aa1n02p5x5 g174(.a(new_n265), .b(new_n269), .c(new_n260), .o1(new_n270));
  inv000aa1d42x5               g175(.a(new_n270), .o1(new_n271));
  aoi012aa1n02x5               g176(.a(new_n271), .b(new_n254), .c(new_n266), .o1(new_n272));
  norp02aa1n24x5               g177(.a(\b[24] ), .b(\a[25] ), .o1(new_n273));
  and002aa1n02x5               g178(.a(\b[24] ), .b(\a[25] ), .o(new_n274));
  nor002aa1n02x5               g179(.a(new_n274), .b(new_n273), .o1(new_n275));
  aob012aa1n03x5               g180(.a(new_n275), .b(new_n268), .c(new_n272), .out0(new_n276));
  aoi112aa1n02x5               g181(.a(new_n275), .b(new_n271), .c(new_n254), .d(new_n266), .o1(new_n277));
  aobi12aa1n02x7               g182(.a(new_n276), .b(new_n277), .c(new_n268), .out0(\s[25] ));
  xorc02aa1n02x5               g183(.a(\a[26] ), .b(\b[25] ), .out0(new_n279));
  norp02aa1n02x5               g184(.a(new_n279), .b(new_n273), .o1(new_n280));
  inv000aa1d42x5               g185(.a(new_n273), .o1(new_n281));
  aoai13aa1n03x5               g186(.a(new_n281), .b(new_n274), .c(new_n268), .d(new_n272), .o1(new_n282));
  aoi022aa1n03x5               g187(.a(new_n282), .b(new_n279), .c(new_n276), .d(new_n280), .o1(\s[26] ));
  and002aa1n02x5               g188(.a(new_n279), .b(new_n275), .o(new_n284));
  aoai13aa1n06x5               g189(.a(new_n284), .b(new_n271), .c(new_n254), .d(new_n266), .o1(new_n285));
  norp02aa1n02x5               g190(.a(\b[26] ), .b(\a[27] ), .o1(new_n286));
  and002aa1n02x5               g191(.a(\b[26] ), .b(\a[27] ), .o(new_n287));
  norp02aa1n02x5               g192(.a(new_n287), .b(new_n286), .o1(new_n288));
  inv000aa1d42x5               g193(.a(new_n249), .o1(new_n289));
  nano32aa1n03x7               g194(.a(new_n234), .b(new_n284), .c(new_n289), .d(new_n266), .out0(new_n290));
  oao003aa1n02x5               g195(.a(\a[26] ), .b(\b[25] ), .c(new_n281), .carry(new_n291));
  inv000aa1d42x5               g196(.a(new_n291), .o1(new_n292));
  aoi112aa1n02x5               g197(.a(new_n292), .b(new_n288), .c(new_n203), .d(new_n290), .o1(new_n293));
  aoi012aa1n09x5               g198(.a(new_n292), .b(new_n203), .c(new_n290), .o1(new_n294));
  nanp02aa1n02x5               g199(.a(new_n294), .b(new_n285), .o1(new_n295));
  aoi022aa1n02x5               g200(.a(new_n295), .b(new_n288), .c(new_n285), .d(new_n293), .o1(\s[27] ));
  aoai13aa1n03x5               g201(.a(new_n266), .b(new_n252), .c(new_n238), .d(new_n289), .o1(new_n297));
  aobi12aa1n06x5               g202(.a(new_n284), .b(new_n297), .c(new_n270), .out0(new_n298));
  nanp02aa1n02x5               g203(.a(\b[26] ), .b(\a[27] ), .o1(new_n299));
  nona23aa1n09x5               g204(.a(new_n179), .b(new_n185), .c(new_n172), .d(new_n164), .out0(new_n300));
  norb02aa1n06x5               g205(.a(new_n154), .b(new_n300), .out0(new_n301));
  oai012aa1n04x7               g206(.a(new_n301), .b(new_n137), .c(new_n123), .o1(new_n302));
  nanp03aa1n02x5               g207(.a(new_n250), .b(new_n266), .c(new_n284), .o1(new_n303));
  aoai13aa1n02x7               g208(.a(new_n291), .b(new_n303), .c(new_n302), .d(new_n202), .o1(new_n304));
  oai012aa1n02x5               g209(.a(new_n299), .b(new_n304), .c(new_n298), .o1(new_n305));
  inv000aa1n03x5               g210(.a(new_n286), .o1(new_n306));
  aoai13aa1n03x5               g211(.a(new_n306), .b(new_n287), .c(new_n294), .d(new_n285), .o1(new_n307));
  xorc02aa1n02x5               g212(.a(\a[28] ), .b(\b[27] ), .out0(new_n308));
  norp02aa1n02x5               g213(.a(new_n308), .b(new_n286), .o1(new_n309));
  aoi022aa1n03x5               g214(.a(new_n307), .b(new_n308), .c(new_n305), .d(new_n309), .o1(\s[28] ));
  inv000aa1d42x5               g215(.a(\a[27] ), .o1(new_n311));
  inv000aa1d42x5               g216(.a(\a[28] ), .o1(new_n312));
  xroi22aa1d06x4               g217(.a(new_n311), .b(\b[26] ), .c(new_n312), .d(\b[27] ), .out0(new_n313));
  oai012aa1n02x5               g218(.a(new_n313), .b(new_n304), .c(new_n298), .o1(new_n314));
  inv000aa1n06x5               g219(.a(new_n313), .o1(new_n315));
  oao003aa1n02x5               g220(.a(\a[28] ), .b(\b[27] ), .c(new_n306), .carry(new_n316));
  aoai13aa1n03x5               g221(.a(new_n316), .b(new_n315), .c(new_n294), .d(new_n285), .o1(new_n317));
  xorc02aa1n02x5               g222(.a(\a[29] ), .b(\b[28] ), .out0(new_n318));
  norb02aa1n02x5               g223(.a(new_n316), .b(new_n318), .out0(new_n319));
  aoi022aa1n03x5               g224(.a(new_n317), .b(new_n318), .c(new_n314), .d(new_n319), .o1(\s[29] ));
  xorb03aa1n02x5               g225(.a(new_n100), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  and003aa1n02x5               g226(.a(new_n308), .b(new_n318), .c(new_n288), .o(new_n322));
  oai012aa1n02x5               g227(.a(new_n322), .b(new_n304), .c(new_n298), .o1(new_n323));
  inv000aa1d42x5               g228(.a(new_n322), .o1(new_n324));
  oao003aa1n02x5               g229(.a(\a[29] ), .b(\b[28] ), .c(new_n316), .carry(new_n325));
  aoai13aa1n03x5               g230(.a(new_n325), .b(new_n324), .c(new_n294), .d(new_n285), .o1(new_n326));
  xorc02aa1n02x5               g231(.a(\a[30] ), .b(\b[29] ), .out0(new_n327));
  norb02aa1n02x5               g232(.a(new_n325), .b(new_n327), .out0(new_n328));
  aoi022aa1n03x5               g233(.a(new_n326), .b(new_n327), .c(new_n323), .d(new_n328), .o1(\s[30] ));
  nano22aa1n02x4               g234(.a(new_n315), .b(new_n318), .c(new_n327), .out0(new_n330));
  tech160nm_fioai012aa1n04x5   g235(.a(new_n330), .b(new_n304), .c(new_n298), .o1(new_n331));
  xorc02aa1n02x5               g236(.a(\a[31] ), .b(\b[30] ), .out0(new_n332));
  and002aa1n02x5               g237(.a(\b[29] ), .b(\a[30] ), .o(new_n333));
  oabi12aa1n02x5               g238(.a(new_n332), .b(\a[30] ), .c(\b[29] ), .out0(new_n334));
  oab012aa1n02x4               g239(.a(new_n334), .b(new_n325), .c(new_n333), .out0(new_n335));
  inv000aa1n02x5               g240(.a(new_n330), .o1(new_n336));
  oao003aa1n02x5               g241(.a(\a[30] ), .b(\b[29] ), .c(new_n325), .carry(new_n337));
  aoai13aa1n03x5               g242(.a(new_n337), .b(new_n336), .c(new_n294), .d(new_n285), .o1(new_n338));
  aoi022aa1n03x5               g243(.a(new_n338), .b(new_n332), .c(new_n331), .d(new_n335), .o1(\s[31] ));
  xnbna2aa1n03x5               g244(.a(new_n132), .b(new_n101), .c(new_n99), .out0(\s[3] ));
  xorc02aa1n02x5               g245(.a(\a[4] ), .b(\b[3] ), .out0(new_n341));
  aoi112aa1n02x5               g246(.a(new_n102), .b(new_n341), .c(new_n131), .d(new_n132), .o1(new_n342));
  aob012aa1n02x5               g247(.a(new_n107), .b(\b[3] ), .c(\a[4] ), .out0(new_n343));
  oab012aa1n02x4               g248(.a(new_n342), .b(new_n343), .c(new_n105), .out0(\s[4] ));
  xnrb03aa1n02x5               g249(.a(new_n343), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oaoi03aa1n02x5               g250(.a(\a[5] ), .b(\b[4] ), .c(new_n343), .o1(new_n346));
  xorb03aa1n02x5               g251(.a(new_n346), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  inv000aa1d42x5               g252(.a(new_n117), .o1(new_n348));
  nanp03aa1n02x5               g253(.a(new_n346), .b(new_n120), .c(new_n348), .o1(new_n349));
  norb02aa1n02x5               g254(.a(new_n116), .b(new_n115), .out0(new_n350));
  xnbna2aa1n03x5               g255(.a(new_n350), .b(new_n349), .c(new_n348), .out0(\s[7] ));
  aoai13aa1n02x5               g256(.a(new_n350), .b(new_n117), .c(new_n346), .d(new_n120), .o1(new_n352));
  xorc02aa1n02x5               g257(.a(\a[8] ), .b(\b[7] ), .out0(new_n353));
  xnbna2aa1n03x5               g258(.a(new_n353), .b(new_n352), .c(new_n192), .out0(\s[8] ));
  xnbna2aa1n03x5               g259(.a(new_n125), .b(new_n191), .c(new_n195), .out0(\s[9] ));
endmodule


