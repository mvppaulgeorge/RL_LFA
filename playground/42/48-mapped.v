// Benchmark "adder" written by ABC on Thu Jul 18 09:56:00 2024

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
    new_n133, new_n134, new_n136, new_n137, new_n138, new_n139, new_n140,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n155, new_n156,
    new_n157, new_n158, new_n159, new_n160, new_n161, new_n162, new_n163,
    new_n164, new_n165, new_n166, new_n167, new_n169, new_n170, new_n171,
    new_n172, new_n173, new_n174, new_n175, new_n176, new_n178, new_n179,
    new_n180, new_n181, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n194, new_n195,
    new_n196, new_n197, new_n198, new_n200, new_n201, new_n202, new_n203,
    new_n204, new_n205, new_n206, new_n207, new_n208, new_n209, new_n210,
    new_n211, new_n214, new_n215, new_n216, new_n217, new_n218, new_n219,
    new_n221, new_n222, new_n223, new_n224, new_n225, new_n226, new_n227,
    new_n228, new_n229, new_n231, new_n232, new_n233, new_n234, new_n235,
    new_n236, new_n237, new_n238, new_n239, new_n241, new_n242, new_n243,
    new_n244, new_n245, new_n246, new_n248, new_n249, new_n250, new_n251,
    new_n252, new_n253, new_n254, new_n255, new_n256, new_n257, new_n258,
    new_n260, new_n261, new_n262, new_n263, new_n264, new_n265, new_n266,
    new_n267, new_n268, new_n269, new_n270, new_n271, new_n272, new_n274,
    new_n275, new_n276, new_n277, new_n278, new_n280, new_n281, new_n282,
    new_n283, new_n284, new_n285, new_n286, new_n287, new_n288, new_n289,
    new_n290, new_n291, new_n292, new_n294, new_n295, new_n296, new_n297,
    new_n298, new_n299, new_n300, new_n301, new_n302, new_n304, new_n305,
    new_n306, new_n307, new_n308, new_n309, new_n310, new_n313, new_n314,
    new_n315, new_n316, new_n317, new_n318, new_n319, new_n321, new_n322,
    new_n323, new_n324, new_n325, new_n326, new_n327, new_n330, new_n331,
    new_n332, new_n335, new_n337, new_n339;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  norp02aa1n02x5               g001(.a(\b[2] ), .b(\a[3] ), .o1(new_n97));
  oab012aa1n02x4               g002(.a(new_n97), .b(\a[4] ), .c(\b[3] ), .out0(new_n98));
  xnrc02aa1n02x5               g003(.a(\b[2] ), .b(\a[3] ), .out0(new_n99));
  inv000aa1d42x5               g004(.a(\a[2] ), .o1(new_n100));
  inv000aa1d42x5               g005(.a(\b[1] ), .o1(new_n101));
  nand02aa1d06x5               g006(.a(\b[0] ), .b(\a[1] ), .o1(new_n102));
  oaoi03aa1n12x5               g007(.a(new_n100), .b(new_n101), .c(new_n102), .o1(new_n103));
  oai012aa1n04x7               g008(.a(new_n98), .b(new_n103), .c(new_n99), .o1(new_n104));
  nand42aa1n04x5               g009(.a(\b[6] ), .b(\a[7] ), .o1(new_n105));
  nor002aa1n08x5               g010(.a(\b[6] ), .b(\a[7] ), .o1(new_n106));
  norb02aa1n09x5               g011(.a(new_n105), .b(new_n106), .out0(new_n107));
  nand42aa1n02x5               g012(.a(\b[3] ), .b(\a[4] ), .o1(new_n108));
  nand22aa1n04x5               g013(.a(\b[7] ), .b(\a[8] ), .o1(new_n109));
  inv040aa1d32x5               g014(.a(\a[8] ), .o1(new_n110));
  inv000aa1d42x5               g015(.a(\b[7] ), .o1(new_n111));
  nand02aa1n08x5               g016(.a(new_n111), .b(new_n110), .o1(new_n112));
  nand23aa1n03x5               g017(.a(new_n112), .b(new_n108), .c(new_n109), .o1(new_n113));
  nor022aa1n04x5               g018(.a(\b[5] ), .b(\a[6] ), .o1(new_n114));
  nand02aa1d06x5               g019(.a(\b[5] ), .b(\a[6] ), .o1(new_n115));
  nanb02aa1n12x5               g020(.a(new_n114), .b(new_n115), .out0(new_n116));
  nor002aa1n03x5               g021(.a(\b[4] ), .b(\a[5] ), .o1(new_n117));
  nand42aa1n03x5               g022(.a(\b[4] ), .b(\a[5] ), .o1(new_n118));
  norb02aa1n06x4               g023(.a(new_n118), .b(new_n117), .out0(new_n119));
  nano23aa1n06x5               g024(.a(new_n116), .b(new_n113), .c(new_n119), .d(new_n107), .out0(new_n120));
  nand02aa1n02x5               g025(.a(new_n117), .b(new_n115), .o1(new_n121));
  nona22aa1n03x5               g026(.a(new_n121), .b(new_n114), .c(new_n106), .out0(new_n122));
  nanp03aa1n03x5               g027(.a(new_n122), .b(new_n105), .c(new_n109), .o1(new_n123));
  nand42aa1n02x5               g028(.a(new_n123), .b(new_n112), .o1(new_n124));
  aoi012aa1n12x5               g029(.a(new_n124), .b(new_n120), .c(new_n104), .o1(new_n125));
  oaoi03aa1n09x5               g030(.a(\a[9] ), .b(\b[8] ), .c(new_n125), .o1(new_n126));
  xorb03aa1n02x5               g031(.a(new_n126), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  nor042aa1n04x5               g032(.a(\b[9] ), .b(\a[10] ), .o1(new_n128));
  nand02aa1d16x5               g033(.a(\b[9] ), .b(\a[10] ), .o1(new_n129));
  xorc02aa1n12x5               g034(.a(\a[11] ), .b(\b[10] ), .out0(new_n130));
  oaoi13aa1n02x5               g035(.a(new_n130), .b(new_n129), .c(new_n126), .d(new_n128), .o1(new_n131));
  norb02aa1n03x5               g036(.a(new_n129), .b(new_n128), .out0(new_n132));
  norb02aa1n03x5               g037(.a(new_n132), .b(new_n126), .out0(new_n133));
  nano22aa1n03x7               g038(.a(new_n133), .b(new_n129), .c(new_n130), .out0(new_n134));
  norp02aa1n02x5               g039(.a(new_n134), .b(new_n131), .o1(\s[11] ));
  orn002aa1n02x7               g040(.a(\a[11] ), .b(\b[10] ), .o(new_n136));
  oai112aa1n02x5               g041(.a(new_n129), .b(new_n130), .c(new_n126), .d(new_n128), .o1(new_n137));
  xnrc02aa1n12x5               g042(.a(\b[11] ), .b(\a[12] ), .out0(new_n138));
  aoi012aa1n02x5               g043(.a(new_n138), .b(new_n137), .c(new_n136), .o1(new_n139));
  nano22aa1n02x4               g044(.a(new_n134), .b(new_n136), .c(new_n138), .out0(new_n140));
  norp02aa1n03x5               g045(.a(new_n139), .b(new_n140), .o1(\s[12] ));
  tech160nm_fixnrc02aa1n02p5x5 g046(.a(\b[8] ), .b(\a[9] ), .out0(new_n142));
  nano23aa1n06x5               g047(.a(new_n142), .b(new_n138), .c(new_n130), .d(new_n132), .out0(new_n143));
  aoai13aa1n06x5               g048(.a(new_n143), .b(new_n124), .c(new_n120), .d(new_n104), .o1(new_n144));
  norp02aa1n02x5               g049(.a(\b[11] ), .b(\a[12] ), .o1(new_n145));
  inv000aa1n02x5               g050(.a(new_n145), .o1(new_n146));
  tech160nm_fiao0012aa1n04x5   g051(.a(new_n136), .b(\a[12] ), .c(\b[11] ), .o(new_n147));
  xnrc02aa1n02x5               g052(.a(\b[10] ), .b(\a[11] ), .out0(new_n148));
  nor042aa1n02x5               g053(.a(\b[8] ), .b(\a[9] ), .o1(new_n149));
  tech160nm_fioai012aa1n02p5x5 g054(.a(new_n129), .b(new_n128), .c(new_n149), .o1(new_n150));
  norp03aa1n02x5               g055(.a(new_n148), .b(new_n138), .c(new_n150), .o1(new_n151));
  nano22aa1n03x5               g056(.a(new_n151), .b(new_n147), .c(new_n146), .out0(new_n152));
  xorc02aa1n12x5               g057(.a(\a[13] ), .b(\b[12] ), .out0(new_n153));
  xnbna2aa1n03x5               g058(.a(new_n153), .b(new_n144), .c(new_n152), .out0(\s[13] ));
  xorc02aa1n02x5               g059(.a(\a[3] ), .b(\b[2] ), .out0(new_n155));
  oao003aa1n02x5               g060(.a(new_n100), .b(new_n101), .c(new_n102), .carry(new_n156));
  aobi12aa1n02x5               g061(.a(new_n98), .b(new_n156), .c(new_n155), .out0(new_n157));
  nona23aa1n02x4               g062(.a(new_n119), .b(new_n107), .c(new_n113), .d(new_n116), .out0(new_n158));
  inv000aa1d42x5               g063(.a(new_n112), .o1(new_n159));
  aoi013aa1n02x4               g064(.a(new_n159), .b(new_n122), .c(new_n105), .d(new_n109), .o1(new_n160));
  nona23aa1n02x4               g065(.a(new_n130), .b(new_n132), .c(new_n138), .d(new_n142), .out0(new_n161));
  oaoi13aa1n02x5               g066(.a(new_n161), .b(new_n160), .c(new_n158), .d(new_n157), .o1(new_n162));
  xorc02aa1n02x5               g067(.a(\a[12] ), .b(\b[11] ), .out0(new_n163));
  nanb03aa1n02x5               g068(.a(new_n150), .b(new_n130), .c(new_n163), .out0(new_n164));
  nand23aa1n03x5               g069(.a(new_n164), .b(new_n146), .c(new_n147), .o1(new_n165));
  norp02aa1n02x5               g070(.a(\b[12] ), .b(\a[13] ), .o1(new_n166));
  oaoi13aa1n03x5               g071(.a(new_n166), .b(new_n153), .c(new_n162), .d(new_n165), .o1(new_n167));
  xnrb03aa1n02x5               g072(.a(new_n167), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  nanp02aa1n02x5               g073(.a(\b[13] ), .b(\a[14] ), .o1(new_n169));
  nanp02aa1n04x5               g074(.a(\b[14] ), .b(\a[15] ), .o1(new_n170));
  nor042aa1n06x5               g075(.a(\b[14] ), .b(\a[15] ), .o1(new_n171));
  nanb02aa1n06x5               g076(.a(new_n171), .b(new_n170), .out0(new_n172));
  inv000aa1d42x5               g077(.a(new_n153), .o1(new_n173));
  oai022aa1d24x5               g078(.a(\a[13] ), .b(\b[12] ), .c(\b[13] ), .d(\a[14] ), .o1(new_n174));
  inv000aa1d42x5               g079(.a(new_n174), .o1(new_n175));
  aoai13aa1n04x5               g080(.a(new_n175), .b(new_n173), .c(new_n144), .d(new_n152), .o1(new_n176));
  xnbna2aa1n03x5               g081(.a(new_n172), .b(new_n176), .c(new_n169), .out0(\s[15] ));
  nano22aa1n02x4               g082(.a(new_n171), .b(new_n169), .c(new_n170), .out0(new_n178));
  tech160nm_fixorc02aa1n04x5   g083(.a(\a[16] ), .b(\b[15] ), .out0(new_n179));
  aoi112aa1n02x5               g084(.a(new_n179), .b(new_n171), .c(new_n176), .d(new_n178), .o1(new_n180));
  aoai13aa1n06x5               g085(.a(new_n179), .b(new_n171), .c(new_n176), .d(new_n178), .o1(new_n181));
  norb02aa1n02x7               g086(.a(new_n181), .b(new_n180), .out0(\s[16] ));
  xnrc02aa1n12x5               g087(.a(\b[13] ), .b(\a[14] ), .out0(new_n183));
  nona23aa1n03x5               g088(.a(new_n179), .b(new_n153), .c(new_n183), .d(new_n172), .out0(new_n184));
  nor042aa1n03x5               g089(.a(new_n161), .b(new_n184), .o1(new_n185));
  aoai13aa1n06x5               g090(.a(new_n185), .b(new_n124), .c(new_n104), .d(new_n120), .o1(new_n186));
  nanp02aa1n02x5               g091(.a(\b[15] ), .b(\a[16] ), .o1(new_n187));
  nano23aa1n09x5               g092(.a(new_n172), .b(new_n183), .c(new_n179), .d(new_n153), .out0(new_n188));
  aoai13aa1n06x5               g093(.a(new_n170), .b(new_n171), .c(new_n174), .d(new_n169), .o1(new_n189));
  oaih12aa1n06x5               g094(.a(new_n189), .b(\b[15] ), .c(\a[16] ), .o1(new_n190));
  aoi022aa1d18x5               g095(.a(new_n165), .b(new_n188), .c(new_n187), .d(new_n190), .o1(new_n191));
  xorc02aa1n12x5               g096(.a(\a[17] ), .b(\b[16] ), .out0(new_n192));
  xnbna2aa1n03x5               g097(.a(new_n192), .b(new_n186), .c(new_n191), .out0(\s[17] ));
  inv000aa1d42x5               g098(.a(\a[17] ), .o1(new_n194));
  inv000aa1d42x5               g099(.a(\b[16] ), .o1(new_n195));
  nand22aa1n03x5               g100(.a(new_n188), .b(new_n143), .o1(new_n196));
  oai012aa1d24x5               g101(.a(new_n191), .b(new_n196), .c(new_n125), .o1(new_n197));
  oaoi03aa1n03x5               g102(.a(new_n194), .b(new_n195), .c(new_n197), .o1(new_n198));
  xnrb03aa1n03x5               g103(.a(new_n198), .b(\b[17] ), .c(\a[18] ), .out0(\s[18] ));
  nand22aa1n03x5               g104(.a(\b[17] ), .b(\a[18] ), .o1(new_n200));
  nand02aa1d06x5               g105(.a(\b[18] ), .b(\a[19] ), .o1(new_n201));
  inv000aa1d42x5               g106(.a(\b[18] ), .o1(new_n202));
  nanb02aa1n09x5               g107(.a(\a[19] ), .b(new_n202), .out0(new_n203));
  inv000aa1d42x5               g108(.a(new_n192), .o1(new_n204));
  oai022aa1d24x5               g109(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n205));
  inv000aa1d42x5               g110(.a(new_n205), .o1(new_n206));
  aoai13aa1n04x5               g111(.a(new_n206), .b(new_n204), .c(new_n186), .d(new_n191), .o1(new_n207));
  aoi022aa1n02x5               g112(.a(new_n207), .b(new_n200), .c(new_n201), .d(new_n203), .o1(new_n208));
  nor002aa1n03x5               g113(.a(\b[18] ), .b(\a[19] ), .o1(new_n209));
  nano22aa1n02x4               g114(.a(new_n209), .b(new_n200), .c(new_n201), .out0(new_n210));
  aoai13aa1n03x5               g115(.a(new_n210), .b(new_n205), .c(new_n197), .d(new_n192), .o1(new_n211));
  norb02aa1n02x5               g116(.a(new_n211), .b(new_n208), .out0(\s[19] ));
  xnrc02aa1n02x5               g117(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor042aa1n06x5               g118(.a(\b[19] ), .b(\a[20] ), .o1(new_n214));
  nand02aa1d10x5               g119(.a(\b[19] ), .b(\a[20] ), .o1(new_n215));
  norb02aa1n02x5               g120(.a(new_n215), .b(new_n214), .out0(new_n216));
  inv000aa1d42x5               g121(.a(new_n216), .o1(new_n217));
  aoi012aa1n03x5               g122(.a(new_n217), .b(new_n211), .c(new_n203), .o1(new_n218));
  aoi112aa1n02x5               g123(.a(new_n216), .b(new_n209), .c(new_n207), .d(new_n210), .o1(new_n219));
  nor042aa1n03x5               g124(.a(new_n218), .b(new_n219), .o1(\s[20] ));
  xnrc02aa1n02x5               g125(.a(\b[17] ), .b(\a[18] ), .out0(new_n221));
  nanb03aa1d18x5               g126(.a(new_n214), .b(new_n215), .c(new_n201), .out0(new_n222));
  nano23aa1n09x5               g127(.a(new_n222), .b(new_n221), .c(new_n192), .d(new_n203), .out0(new_n223));
  inv000aa1n02x5               g128(.a(new_n223), .o1(new_n224));
  nand23aa1n06x5               g129(.a(new_n205), .b(new_n203), .c(new_n200), .o1(new_n225));
  aoi012aa1d18x5               g130(.a(new_n214), .b(new_n209), .c(new_n215), .o1(new_n226));
  oai012aa1d24x5               g131(.a(new_n226), .b(new_n225), .c(new_n222), .o1(new_n227));
  inv000aa1d42x5               g132(.a(new_n227), .o1(new_n228));
  aoai13aa1n04x5               g133(.a(new_n228), .b(new_n224), .c(new_n186), .d(new_n191), .o1(new_n229));
  xorb03aa1n02x5               g134(.a(new_n229), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor042aa1d18x5               g135(.a(\b[20] ), .b(\a[21] ), .o1(new_n231));
  xnrc02aa1n12x5               g136(.a(\b[20] ), .b(\a[21] ), .out0(new_n232));
  inv000aa1d42x5               g137(.a(new_n232), .o1(new_n233));
  xnrc02aa1n12x5               g138(.a(\b[21] ), .b(\a[22] ), .out0(new_n234));
  inv000aa1d42x5               g139(.a(new_n234), .o1(new_n235));
  aoi112aa1n02x5               g140(.a(new_n231), .b(new_n235), .c(new_n229), .d(new_n233), .o1(new_n236));
  inv000aa1n06x5               g141(.a(new_n231), .o1(new_n237));
  aoai13aa1n03x5               g142(.a(new_n233), .b(new_n227), .c(new_n197), .d(new_n223), .o1(new_n238));
  aoi012aa1n03x5               g143(.a(new_n234), .b(new_n238), .c(new_n237), .o1(new_n239));
  norp02aa1n03x5               g144(.a(new_n239), .b(new_n236), .o1(\s[22] ));
  nor042aa1n06x5               g145(.a(new_n234), .b(new_n232), .o1(new_n241));
  nand22aa1n12x5               g146(.a(new_n223), .b(new_n241), .o1(new_n242));
  oao003aa1n02x5               g147(.a(\a[22] ), .b(\b[21] ), .c(new_n237), .carry(new_n243));
  inv020aa1n02x5               g148(.a(new_n243), .o1(new_n244));
  aoi012aa1d18x5               g149(.a(new_n244), .b(new_n227), .c(new_n241), .o1(new_n245));
  aoai13aa1n04x5               g150(.a(new_n245), .b(new_n242), .c(new_n186), .d(new_n191), .o1(new_n246));
  xorb03aa1n02x5               g151(.a(new_n246), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  nor042aa1n04x5               g152(.a(\b[22] ), .b(\a[23] ), .o1(new_n248));
  and002aa1n18x5               g153(.a(\b[22] ), .b(\a[23] ), .o(new_n249));
  norp02aa1n02x5               g154(.a(new_n249), .b(new_n248), .o1(new_n250));
  xorc02aa1n06x5               g155(.a(\a[24] ), .b(\b[23] ), .out0(new_n251));
  aoi112aa1n02x5               g156(.a(new_n248), .b(new_n251), .c(new_n246), .d(new_n250), .o1(new_n252));
  inv000aa1d42x5               g157(.a(new_n248), .o1(new_n253));
  inv000aa1d42x5               g158(.a(new_n242), .o1(new_n254));
  inv000aa1d42x5               g159(.a(new_n245), .o1(new_n255));
  aoai13aa1n03x5               g160(.a(new_n250), .b(new_n255), .c(new_n197), .d(new_n254), .o1(new_n256));
  inv000aa1d42x5               g161(.a(new_n251), .o1(new_n257));
  aoi012aa1n02x7               g162(.a(new_n257), .b(new_n256), .c(new_n253), .o1(new_n258));
  nor002aa1n02x5               g163(.a(new_n258), .b(new_n252), .o1(\s[24] ));
  tech160nm_fioai012aa1n03p5x5 g164(.a(new_n160), .b(new_n158), .c(new_n157), .o1(new_n260));
  nanp02aa1n02x5               g165(.a(new_n190), .b(new_n187), .o1(new_n261));
  oai012aa1n02x5               g166(.a(new_n261), .b(new_n152), .c(new_n184), .o1(new_n262));
  nano32aa1n03x7               g167(.a(new_n224), .b(new_n251), .c(new_n241), .d(new_n250), .out0(new_n263));
  aoai13aa1n03x5               g168(.a(new_n263), .b(new_n262), .c(new_n260), .d(new_n185), .o1(new_n264));
  inv000aa1d42x5               g169(.a(new_n249), .o1(new_n265));
  aoai13aa1n09x5               g170(.a(new_n265), .b(new_n244), .c(new_n227), .d(new_n241), .o1(new_n266));
  oab012aa1n04x5               g171(.a(new_n248), .b(\a[24] ), .c(\b[23] ), .out0(new_n267));
  aoi022aa1n03x5               g172(.a(new_n266), .b(new_n267), .c(\b[23] ), .d(\a[24] ), .o1(new_n268));
  xnrc02aa1n12x5               g173(.a(\b[24] ), .b(\a[25] ), .out0(new_n269));
  aoib12aa1n06x5               g174(.a(new_n269), .b(new_n264), .c(new_n268), .out0(new_n270));
  inv000aa1d42x5               g175(.a(new_n269), .o1(new_n271));
  aoi112aa1n02x5               g176(.a(new_n271), .b(new_n268), .c(new_n197), .d(new_n263), .o1(new_n272));
  norp02aa1n02x5               g177(.a(new_n270), .b(new_n272), .o1(\s[25] ));
  orn002aa1n02x5               g178(.a(\a[25] ), .b(\b[24] ), .o(new_n274));
  aoai13aa1n03x5               g179(.a(new_n271), .b(new_n268), .c(new_n197), .d(new_n263), .o1(new_n275));
  xnrc02aa1n12x5               g180(.a(\b[25] ), .b(\a[26] ), .out0(new_n276));
  aoi012aa1n03x5               g181(.a(new_n276), .b(new_n275), .c(new_n274), .o1(new_n277));
  nano22aa1n03x5               g182(.a(new_n270), .b(new_n274), .c(new_n276), .out0(new_n278));
  norp02aa1n03x5               g183(.a(new_n277), .b(new_n278), .o1(\s[26] ));
  and002aa1n02x5               g184(.a(\b[23] ), .b(\a[24] ), .o(new_n280));
  nor043aa1n09x5               g185(.a(new_n276), .b(new_n269), .c(new_n280), .o1(new_n281));
  nano32aa1d12x5               g186(.a(new_n242), .b(new_n281), .c(new_n250), .d(new_n251), .out0(new_n282));
  aoai13aa1n06x5               g187(.a(new_n282), .b(new_n262), .c(new_n260), .d(new_n185), .o1(new_n283));
  nano22aa1n02x4               g188(.a(new_n214), .b(new_n201), .c(new_n215), .out0(new_n284));
  oai012aa1n02x5               g189(.a(new_n200), .b(\b[18] ), .c(\a[19] ), .o1(new_n285));
  norb02aa1n02x5               g190(.a(new_n205), .b(new_n285), .out0(new_n286));
  inv000aa1d42x5               g191(.a(new_n226), .o1(new_n287));
  aoai13aa1n06x5               g192(.a(new_n241), .b(new_n287), .c(new_n286), .d(new_n284), .o1(new_n288));
  aoai13aa1n04x5               g193(.a(new_n267), .b(new_n249), .c(new_n288), .d(new_n243), .o1(new_n289));
  oao003aa1n02x5               g194(.a(\a[26] ), .b(\b[25] ), .c(new_n274), .carry(new_n290));
  aobi12aa1n06x5               g195(.a(new_n290), .b(new_n289), .c(new_n281), .out0(new_n291));
  xorc02aa1n12x5               g196(.a(\a[27] ), .b(\b[26] ), .out0(new_n292));
  xnbna2aa1n03x5               g197(.a(new_n292), .b(new_n291), .c(new_n283), .out0(\s[27] ));
  norp02aa1n02x5               g198(.a(\b[26] ), .b(\a[27] ), .o1(new_n294));
  inv040aa1n03x5               g199(.a(new_n294), .o1(new_n295));
  aobi12aa1n02x7               g200(.a(new_n292), .b(new_n291), .c(new_n283), .out0(new_n296));
  xnrc02aa1n02x5               g201(.a(\b[27] ), .b(\a[28] ), .out0(new_n297));
  nano22aa1n03x5               g202(.a(new_n296), .b(new_n295), .c(new_n297), .out0(new_n298));
  inv000aa1d42x5               g203(.a(new_n281), .o1(new_n299));
  aoai13aa1n06x5               g204(.a(new_n290), .b(new_n299), .c(new_n266), .d(new_n267), .o1(new_n300));
  aoai13aa1n03x5               g205(.a(new_n292), .b(new_n300), .c(new_n197), .d(new_n282), .o1(new_n301));
  tech160nm_fiaoi012aa1n02p5x5 g206(.a(new_n297), .b(new_n301), .c(new_n295), .o1(new_n302));
  norp02aa1n03x5               g207(.a(new_n302), .b(new_n298), .o1(\s[28] ));
  norb02aa1n02x5               g208(.a(new_n292), .b(new_n297), .out0(new_n304));
  aoai13aa1n03x5               g209(.a(new_n304), .b(new_n300), .c(new_n197), .d(new_n282), .o1(new_n305));
  oao003aa1n02x5               g210(.a(\a[28] ), .b(\b[27] ), .c(new_n295), .carry(new_n306));
  xnrc02aa1n02x5               g211(.a(\b[28] ), .b(\a[29] ), .out0(new_n307));
  tech160nm_fiaoi012aa1n02p5x5 g212(.a(new_n307), .b(new_n305), .c(new_n306), .o1(new_n308));
  aobi12aa1n02x7               g213(.a(new_n304), .b(new_n291), .c(new_n283), .out0(new_n309));
  nano22aa1n03x5               g214(.a(new_n309), .b(new_n306), .c(new_n307), .out0(new_n310));
  norp02aa1n03x5               g215(.a(new_n308), .b(new_n310), .o1(\s[29] ));
  xorb03aa1n02x5               g216(.a(new_n102), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g217(.a(new_n292), .b(new_n307), .c(new_n297), .out0(new_n313));
  aoai13aa1n03x5               g218(.a(new_n313), .b(new_n300), .c(new_n197), .d(new_n282), .o1(new_n314));
  oao003aa1n02x5               g219(.a(\a[29] ), .b(\b[28] ), .c(new_n306), .carry(new_n315));
  xnrc02aa1n02x5               g220(.a(\b[29] ), .b(\a[30] ), .out0(new_n316));
  tech160nm_fiaoi012aa1n02p5x5 g221(.a(new_n316), .b(new_n314), .c(new_n315), .o1(new_n317));
  aobi12aa1n02x7               g222(.a(new_n313), .b(new_n291), .c(new_n283), .out0(new_n318));
  nano22aa1n03x5               g223(.a(new_n318), .b(new_n315), .c(new_n316), .out0(new_n319));
  norp02aa1n03x5               g224(.a(new_n317), .b(new_n319), .o1(\s[30] ));
  xnrc02aa1n02x5               g225(.a(\b[30] ), .b(\a[31] ), .out0(new_n321));
  norb02aa1n02x5               g226(.a(new_n313), .b(new_n316), .out0(new_n322));
  aoai13aa1n03x5               g227(.a(new_n322), .b(new_n300), .c(new_n197), .d(new_n282), .o1(new_n323));
  oao003aa1n02x5               g228(.a(\a[30] ), .b(\b[29] ), .c(new_n315), .carry(new_n324));
  tech160nm_fiaoi012aa1n02p5x5 g229(.a(new_n321), .b(new_n323), .c(new_n324), .o1(new_n325));
  aobi12aa1n02x7               g230(.a(new_n322), .b(new_n291), .c(new_n283), .out0(new_n326));
  nano22aa1n03x5               g231(.a(new_n326), .b(new_n321), .c(new_n324), .out0(new_n327));
  norp02aa1n03x5               g232(.a(new_n325), .b(new_n327), .o1(\s[31] ));
  xnrc02aa1n02x5               g233(.a(new_n103), .b(new_n155), .out0(\s[3] ));
  xorc02aa1n02x5               g234(.a(\a[4] ), .b(\b[3] ), .out0(new_n330));
  aoi112aa1n02x5               g235(.a(new_n330), .b(new_n97), .c(new_n156), .d(new_n155), .o1(new_n331));
  norb02aa1n02x5               g236(.a(new_n108), .b(new_n157), .out0(new_n332));
  oaoi13aa1n02x5               g237(.a(new_n331), .b(new_n332), .c(\a[4] ), .d(\b[3] ), .o1(\s[4] ));
  xobna2aa1n03x5               g238(.a(new_n119), .b(new_n104), .c(new_n108), .out0(\s[5] ));
  aoi013aa1n03x5               g239(.a(new_n117), .b(new_n104), .c(new_n108), .d(new_n118), .o1(new_n335));
  xnrb03aa1n02x5               g240(.a(new_n335), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  nanb02aa1n02x5               g241(.a(new_n116), .b(new_n335), .out0(new_n337));
  xobna2aa1n03x5               g242(.a(new_n107), .b(new_n337), .c(new_n115), .out0(\s[7] ));
  aoi013aa1n03x5               g243(.a(new_n106), .b(new_n337), .c(new_n115), .d(new_n105), .o1(new_n339));
  xnbna2aa1n03x5               g244(.a(new_n339), .b(new_n109), .c(new_n112), .out0(\s[8] ));
  xnrb03aa1n02x5               g245(.a(new_n125), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


