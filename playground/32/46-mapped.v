// Benchmark "adder" written by ABC on Thu Jul 18 04:47:12 2024

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
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n139, new_n140,
    new_n141, new_n142, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n158, new_n159, new_n160, new_n161, new_n163, new_n164,
    new_n165, new_n166, new_n167, new_n168, new_n169, new_n170, new_n171,
    new_n172, new_n173, new_n175, new_n176, new_n177, new_n178, new_n179,
    new_n180, new_n181, new_n182, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n196, new_n197, new_n199, new_n200, new_n201, new_n202, new_n203,
    new_n204, new_n205, new_n206, new_n209, new_n210, new_n211, new_n212,
    new_n213, new_n214, new_n215, new_n216, new_n217, new_n218, new_n219,
    new_n221, new_n222, new_n223, new_n224, new_n225, new_n226, new_n227,
    new_n228, new_n229, new_n231, new_n232, new_n233, new_n234, new_n235,
    new_n237, new_n238, new_n239, new_n240, new_n241, new_n242, new_n243,
    new_n244, new_n245, new_n246, new_n247, new_n249, new_n250, new_n251,
    new_n252, new_n253, new_n254, new_n255, new_n256, new_n257, new_n259,
    new_n260, new_n261, new_n262, new_n263, new_n264, new_n265, new_n266,
    new_n267, new_n269, new_n270, new_n271, new_n272, new_n273, new_n274,
    new_n275, new_n276, new_n278, new_n279, new_n280, new_n281, new_n282,
    new_n283, new_n284, new_n285, new_n287, new_n288, new_n289, new_n290,
    new_n291, new_n292, new_n293, new_n294, new_n295, new_n297, new_n298,
    new_n299, new_n300, new_n301, new_n302, new_n303, new_n306, new_n307,
    new_n308, new_n309, new_n310, new_n311, new_n312, new_n314, new_n315,
    new_n316, new_n317, new_n318, new_n319, new_n320, new_n323, new_n324,
    new_n325, new_n326, new_n329, new_n331, new_n332, new_n334;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor042aa1n09x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(new_n97), .o1(new_n98));
  inv040aa1d28x5               g003(.a(\a[8] ), .o1(new_n99));
  inv000aa1d42x5               g004(.a(\b[7] ), .o1(new_n100));
  nor002aa1n03x5               g005(.a(\b[6] ), .b(\a[7] ), .o1(new_n101));
  aoi012aa1n02x5               g006(.a(new_n101), .b(new_n99), .c(new_n100), .o1(new_n102));
  nanp02aa1n04x5               g007(.a(\b[5] ), .b(\a[6] ), .o1(new_n103));
  nand42aa1n06x5               g008(.a(\b[6] ), .b(\a[7] ), .o1(new_n104));
  nor042aa1n06x5               g009(.a(\b[5] ), .b(\a[6] ), .o1(new_n105));
  nor042aa1n06x5               g010(.a(\b[4] ), .b(\a[5] ), .o1(new_n106));
  oai112aa1n02x5               g011(.a(new_n103), .b(new_n104), .c(new_n105), .d(new_n106), .o1(new_n107));
  aoi022aa1n02x7               g012(.a(new_n107), .b(new_n102), .c(\a[8] ), .d(\b[7] ), .o1(new_n108));
  tech160nm_fixnrc02aa1n04x5   g013(.a(\b[2] ), .b(\a[3] ), .out0(new_n109));
  orn002aa1n02x5               g014(.a(\a[2] ), .b(\b[1] ), .o(new_n110));
  nand42aa1n04x5               g015(.a(\b[0] ), .b(\a[1] ), .o1(new_n111));
  aob012aa1n02x5               g016(.a(new_n111), .b(\b[1] ), .c(\a[2] ), .out0(new_n112));
  inv000aa1d42x5               g017(.a(\a[3] ), .o1(new_n113));
  inv000aa1d42x5               g018(.a(\b[2] ), .o1(new_n114));
  norp02aa1n02x5               g019(.a(\b[3] ), .b(\a[4] ), .o1(new_n115));
  aoi012aa1n02x5               g020(.a(new_n115), .b(new_n113), .c(new_n114), .o1(new_n116));
  aoai13aa1n04x5               g021(.a(new_n116), .b(new_n109), .c(new_n112), .d(new_n110), .o1(new_n117));
  nanp02aa1n02x5               g022(.a(\b[7] ), .b(\a[8] ), .o1(new_n118));
  norb02aa1n03x5               g023(.a(new_n104), .b(new_n101), .out0(new_n119));
  aoi022aa1n02x5               g024(.a(new_n100), .b(new_n99), .c(\a[4] ), .d(\b[3] ), .o1(new_n120));
  nanp02aa1n02x5               g025(.a(\b[4] ), .b(\a[5] ), .o1(new_n121));
  nona23aa1n02x4               g026(.a(new_n103), .b(new_n121), .c(new_n106), .d(new_n105), .out0(new_n122));
  nano32aa1n03x7               g027(.a(new_n122), .b(new_n119), .c(new_n120), .d(new_n118), .out0(new_n123));
  nand42aa1n03x5               g028(.a(\b[8] ), .b(\a[9] ), .o1(new_n124));
  norb02aa1n02x5               g029(.a(new_n124), .b(new_n97), .out0(new_n125));
  aoai13aa1n02x5               g030(.a(new_n125), .b(new_n108), .c(new_n123), .d(new_n117), .o1(new_n126));
  norp02aa1n02x5               g031(.a(\b[9] ), .b(\a[10] ), .o1(new_n127));
  nand42aa1n10x5               g032(.a(\b[9] ), .b(\a[10] ), .o1(new_n128));
  norb02aa1n06x4               g033(.a(new_n128), .b(new_n127), .out0(new_n129));
  xnbna2aa1n03x5               g034(.a(new_n129), .b(new_n126), .c(new_n98), .out0(\s[10] ));
  nor042aa1n04x5               g035(.a(\b[10] ), .b(\a[11] ), .o1(new_n131));
  inv000aa1n02x5               g036(.a(new_n131), .o1(new_n132));
  tech160nm_finand02aa1n03p5x5 g037(.a(\b[10] ), .b(\a[11] ), .o1(new_n133));
  oaoi03aa1n02x5               g038(.a(\a[10] ), .b(\b[9] ), .c(new_n98), .o1(new_n134));
  aoi012aa1n02x5               g039(.a(new_n108), .b(new_n123), .c(new_n117), .o1(new_n135));
  nano22aa1n02x4               g040(.a(new_n135), .b(new_n125), .c(new_n129), .out0(new_n136));
  norp02aa1n02x5               g041(.a(new_n136), .b(new_n134), .o1(new_n137));
  xnbna2aa1n03x5               g042(.a(new_n137), .b(new_n132), .c(new_n133), .out0(\s[11] ));
  nor042aa1n09x5               g043(.a(\b[11] ), .b(\a[12] ), .o1(new_n139));
  inv000aa1d42x5               g044(.a(new_n139), .o1(new_n140));
  nand42aa1n03x5               g045(.a(\b[11] ), .b(\a[12] ), .o1(new_n141));
  oaoi13aa1n02x5               g046(.a(new_n131), .b(new_n133), .c(new_n136), .d(new_n134), .o1(new_n142));
  xnbna2aa1n03x5               g047(.a(new_n142), .b(new_n140), .c(new_n141), .out0(\s[12] ));
  norb03aa1n12x5               g048(.a(new_n124), .b(new_n97), .c(new_n131), .out0(new_n144));
  nano22aa1n03x7               g049(.a(new_n139), .b(new_n133), .c(new_n141), .out0(new_n145));
  nand23aa1d12x5               g050(.a(new_n145), .b(new_n144), .c(new_n129), .o1(new_n146));
  inv000aa1d42x5               g051(.a(new_n146), .o1(new_n147));
  aoai13aa1n02x5               g052(.a(new_n147), .b(new_n108), .c(new_n123), .d(new_n117), .o1(new_n148));
  oai022aa1n02x5               g053(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n149));
  nanb03aa1n02x5               g054(.a(new_n139), .b(new_n141), .c(new_n133), .out0(new_n150));
  nano32aa1n03x5               g055(.a(new_n150), .b(new_n149), .c(new_n132), .d(new_n128), .out0(new_n151));
  aob012aa1n06x5               g056(.a(new_n140), .b(new_n131), .c(new_n141), .out0(new_n152));
  norp02aa1n02x5               g057(.a(new_n151), .b(new_n152), .o1(new_n153));
  norp02aa1n02x5               g058(.a(\b[12] ), .b(\a[13] ), .o1(new_n154));
  nand42aa1n02x5               g059(.a(\b[12] ), .b(\a[13] ), .o1(new_n155));
  nanb02aa1n02x5               g060(.a(new_n154), .b(new_n155), .out0(new_n156));
  xobna2aa1n03x5               g061(.a(new_n156), .b(new_n148), .c(new_n153), .out0(\s[13] ));
  inv000aa1d42x5               g062(.a(\a[13] ), .o1(new_n158));
  inv000aa1d42x5               g063(.a(\b[12] ), .o1(new_n159));
  nanp02aa1n02x5               g064(.a(new_n148), .b(new_n153), .o1(new_n160));
  oaoi03aa1n02x5               g065(.a(new_n158), .b(new_n159), .c(new_n160), .o1(new_n161));
  xnrb03aa1n02x5               g066(.a(new_n161), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  inv040aa1d32x5               g067(.a(\a[15] ), .o1(new_n163));
  inv000aa1d42x5               g068(.a(\b[14] ), .o1(new_n164));
  nand02aa1n08x5               g069(.a(new_n164), .b(new_n163), .o1(new_n165));
  nanp02aa1n04x5               g070(.a(\b[14] ), .b(\a[15] ), .o1(new_n166));
  nand22aa1n06x5               g071(.a(new_n165), .b(new_n166), .o1(new_n167));
  inv000aa1d42x5               g072(.a(new_n167), .o1(new_n168));
  nor022aa1n04x5               g073(.a(\b[13] ), .b(\a[14] ), .o1(new_n169));
  nand02aa1n03x5               g074(.a(\b[13] ), .b(\a[14] ), .o1(new_n170));
  aoai13aa1n04x5               g075(.a(new_n170), .b(new_n169), .c(new_n158), .d(new_n159), .o1(new_n171));
  nanb02aa1n02x5               g076(.a(new_n169), .b(new_n170), .out0(new_n172));
  nona22aa1n02x4               g077(.a(new_n160), .b(new_n156), .c(new_n172), .out0(new_n173));
  xnbna2aa1n03x5               g078(.a(new_n168), .b(new_n173), .c(new_n171), .out0(\s[15] ));
  aoi012aa1n02x5               g079(.a(new_n167), .b(new_n173), .c(new_n171), .o1(new_n175));
  orn002aa1n02x5               g080(.a(\a[16] ), .b(\b[15] ), .o(new_n176));
  nanp02aa1n02x5               g081(.a(\b[15] ), .b(\a[16] ), .o1(new_n177));
  nanp02aa1n02x5               g082(.a(new_n176), .b(new_n177), .o1(new_n178));
  nano22aa1n02x4               g083(.a(new_n175), .b(new_n165), .c(new_n178), .out0(new_n179));
  aoi112aa1n02x5               g084(.a(new_n172), .b(new_n156), .c(new_n148), .d(new_n153), .o1(new_n180));
  oaib12aa1n02x5               g085(.a(new_n168), .b(new_n180), .c(new_n171), .out0(new_n181));
  aoi012aa1n02x5               g086(.a(new_n178), .b(new_n181), .c(new_n165), .o1(new_n182));
  norp02aa1n02x5               g087(.a(new_n182), .b(new_n179), .o1(\s[16] ));
  nano23aa1n02x4               g088(.a(new_n154), .b(new_n169), .c(new_n170), .d(new_n155), .out0(new_n184));
  nona22aa1n02x4               g089(.a(new_n184), .b(new_n178), .c(new_n167), .out0(new_n185));
  nor042aa1n09x5               g090(.a(new_n185), .b(new_n146), .o1(new_n186));
  aoai13aa1n12x5               g091(.a(new_n186), .b(new_n108), .c(new_n123), .d(new_n117), .o1(new_n187));
  nona23aa1n03x5               g092(.a(new_n170), .b(new_n155), .c(new_n154), .d(new_n169), .out0(new_n188));
  nor003aa1n02x5               g093(.a(new_n188), .b(new_n178), .c(new_n167), .o1(new_n189));
  nanp02aa1n02x5               g094(.a(new_n177), .b(new_n166), .o1(new_n190));
  aoai13aa1n02x5               g095(.a(new_n176), .b(new_n190), .c(new_n171), .d(new_n165), .o1(new_n191));
  oaoi13aa1n09x5               g096(.a(new_n191), .b(new_n189), .c(new_n151), .d(new_n152), .o1(new_n192));
  xnrc02aa1n12x5               g097(.a(\b[16] ), .b(\a[17] ), .out0(new_n193));
  inv000aa1n02x5               g098(.a(new_n193), .o1(new_n194));
  xnbna2aa1n03x5               g099(.a(new_n194), .b(new_n187), .c(new_n192), .out0(\s[17] ));
  orn002aa1n02x5               g100(.a(\a[17] ), .b(\b[16] ), .o(new_n196));
  aoai13aa1n02x5               g101(.a(new_n196), .b(new_n193), .c(new_n187), .d(new_n192), .o1(new_n197));
  xorb03aa1n02x5               g102(.a(new_n197), .b(\b[17] ), .c(\a[18] ), .out0(\s[18] ));
  nor042aa1n02x5               g103(.a(\b[18] ), .b(\a[19] ), .o1(new_n199));
  nand42aa1n03x5               g104(.a(\b[18] ), .b(\a[19] ), .o1(new_n200));
  norb02aa1n02x5               g105(.a(new_n200), .b(new_n199), .out0(new_n201));
  oaoi03aa1n02x5               g106(.a(\a[18] ), .b(\b[17] ), .c(new_n196), .o1(new_n202));
  inv000aa1d42x5               g107(.a(new_n202), .o1(new_n203));
  nand02aa1d08x5               g108(.a(new_n187), .b(new_n192), .o1(new_n204));
  xnrc02aa1n02x5               g109(.a(\b[17] ), .b(\a[18] ), .out0(new_n205));
  nona22aa1n03x5               g110(.a(new_n204), .b(new_n193), .c(new_n205), .out0(new_n206));
  xnbna2aa1n03x5               g111(.a(new_n201), .b(new_n206), .c(new_n203), .out0(\s[19] ));
  xnrc02aa1n02x5               g112(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  inv000aa1d42x5               g113(.a(\a[19] ), .o1(new_n209));
  inv000aa1d42x5               g114(.a(\b[18] ), .o1(new_n210));
  tech160nm_finand02aa1n03p5x5 g115(.a(new_n210), .b(new_n209), .o1(new_n211));
  aobi12aa1n03x5               g116(.a(new_n201), .b(new_n206), .c(new_n203), .out0(new_n212));
  nor022aa1n06x5               g117(.a(\b[19] ), .b(\a[20] ), .o1(new_n213));
  tech160nm_finand02aa1n03p5x5 g118(.a(\b[19] ), .b(\a[20] ), .o1(new_n214));
  nanb02aa1n02x5               g119(.a(new_n213), .b(new_n214), .out0(new_n215));
  nano22aa1n03x5               g120(.a(new_n212), .b(new_n211), .c(new_n215), .out0(new_n216));
  aoi112aa1n03x5               g121(.a(new_n205), .b(new_n193), .c(new_n187), .d(new_n192), .o1(new_n217));
  oai012aa1n04x7               g122(.a(new_n201), .b(new_n217), .c(new_n202), .o1(new_n218));
  tech160nm_fiaoi012aa1n03p5x5 g123(.a(new_n215), .b(new_n218), .c(new_n211), .o1(new_n219));
  norp02aa1n02x5               g124(.a(new_n219), .b(new_n216), .o1(\s[20] ));
  nanb03aa1n06x5               g125(.a(new_n213), .b(new_n214), .c(new_n200), .out0(new_n221));
  nona23aa1n03x5               g126(.a(new_n211), .b(new_n194), .c(new_n221), .d(new_n205), .out0(new_n222));
  nanp02aa1n02x5               g127(.a(\b[17] ), .b(\a[18] ), .o1(new_n223));
  oaih22aa1n04x5               g128(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n224));
  nanp03aa1n03x5               g129(.a(new_n224), .b(new_n211), .c(new_n223), .o1(new_n225));
  tech160nm_fiaoi012aa1n03p5x5 g130(.a(new_n213), .b(new_n199), .c(new_n214), .o1(new_n226));
  oai012aa1n12x5               g131(.a(new_n226), .b(new_n225), .c(new_n221), .o1(new_n227));
  inv000aa1d42x5               g132(.a(new_n227), .o1(new_n228));
  aoai13aa1n03x5               g133(.a(new_n228), .b(new_n222), .c(new_n187), .d(new_n192), .o1(new_n229));
  xorb03aa1n02x5               g134(.a(new_n229), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor002aa1n02x5               g135(.a(\b[20] ), .b(\a[21] ), .o1(new_n231));
  xorc02aa1n02x5               g136(.a(\a[21] ), .b(\b[20] ), .out0(new_n232));
  xorc02aa1n02x5               g137(.a(\a[22] ), .b(\b[21] ), .out0(new_n233));
  aoi112aa1n02x5               g138(.a(new_n231), .b(new_n233), .c(new_n229), .d(new_n232), .o1(new_n234));
  aoai13aa1n03x5               g139(.a(new_n233), .b(new_n231), .c(new_n229), .d(new_n232), .o1(new_n235));
  norb02aa1n03x4               g140(.a(new_n235), .b(new_n234), .out0(\s[22] ));
  inv030aa1d32x5               g141(.a(\a[21] ), .o1(new_n237));
  inv040aa1d32x5               g142(.a(\a[22] ), .o1(new_n238));
  xroi22aa1d06x4               g143(.a(new_n237), .b(\b[20] ), .c(new_n238), .d(\b[21] ), .out0(new_n239));
  inv000aa1d42x5               g144(.a(new_n239), .o1(new_n240));
  nona22aa1n03x5               g145(.a(new_n204), .b(new_n222), .c(new_n240), .out0(new_n241));
  inv000aa1d42x5               g146(.a(\b[21] ), .o1(new_n242));
  oaoi03aa1n02x5               g147(.a(new_n238), .b(new_n242), .c(new_n231), .o1(new_n243));
  inv000aa1n02x5               g148(.a(new_n243), .o1(new_n244));
  tech160nm_fiaoi012aa1n04x5   g149(.a(new_n244), .b(new_n227), .c(new_n239), .o1(new_n245));
  xnrc02aa1n12x5               g150(.a(\b[22] ), .b(\a[23] ), .out0(new_n246));
  inv000aa1d42x5               g151(.a(new_n246), .o1(new_n247));
  xnbna2aa1n03x5               g152(.a(new_n247), .b(new_n241), .c(new_n245), .out0(\s[23] ));
  nor042aa1n03x5               g153(.a(\b[22] ), .b(\a[23] ), .o1(new_n249));
  inv000aa1d42x5               g154(.a(new_n249), .o1(new_n250));
  tech160nm_fiaoi012aa1n02p5x5 g155(.a(new_n246), .b(new_n241), .c(new_n245), .o1(new_n251));
  xnrc02aa1n02x5               g156(.a(\b[23] ), .b(\a[24] ), .out0(new_n252));
  nano22aa1n03x5               g157(.a(new_n251), .b(new_n250), .c(new_n252), .out0(new_n253));
  aoi112aa1n02x5               g158(.a(new_n240), .b(new_n222), .c(new_n187), .d(new_n192), .o1(new_n254));
  inv000aa1n02x5               g159(.a(new_n245), .o1(new_n255));
  oai012aa1n02x5               g160(.a(new_n247), .b(new_n254), .c(new_n255), .o1(new_n256));
  aoi012aa1n02x7               g161(.a(new_n252), .b(new_n256), .c(new_n250), .o1(new_n257));
  norp02aa1n02x5               g162(.a(new_n257), .b(new_n253), .o1(\s[24] ));
  norp02aa1n02x5               g163(.a(new_n252), .b(new_n246), .o1(new_n259));
  nanp02aa1n02x5               g164(.a(new_n239), .b(new_n259), .o1(new_n260));
  nona22aa1n03x5               g165(.a(new_n204), .b(new_n222), .c(new_n260), .out0(new_n261));
  aoai13aa1n06x5               g166(.a(new_n259), .b(new_n244), .c(new_n227), .d(new_n239), .o1(new_n262));
  oao003aa1n02x5               g167(.a(\a[24] ), .b(\b[23] ), .c(new_n250), .carry(new_n263));
  nanp02aa1n09x5               g168(.a(new_n262), .b(new_n263), .o1(new_n264));
  inv000aa1n02x5               g169(.a(new_n264), .o1(new_n265));
  xnrc02aa1n12x5               g170(.a(\b[24] ), .b(\a[25] ), .out0(new_n266));
  inv000aa1d42x5               g171(.a(new_n266), .o1(new_n267));
  xnbna2aa1n03x5               g172(.a(new_n267), .b(new_n261), .c(new_n265), .out0(\s[25] ));
  nor042aa1n03x5               g173(.a(\b[24] ), .b(\a[25] ), .o1(new_n269));
  inv000aa1d42x5               g174(.a(new_n269), .o1(new_n270));
  tech160nm_fiaoi012aa1n05x5   g175(.a(new_n266), .b(new_n261), .c(new_n265), .o1(new_n271));
  xnrc02aa1n02x5               g176(.a(\b[25] ), .b(\a[26] ), .out0(new_n272));
  nano22aa1n02x4               g177(.a(new_n271), .b(new_n270), .c(new_n272), .out0(new_n273));
  aoi112aa1n02x5               g178(.a(new_n260), .b(new_n222), .c(new_n187), .d(new_n192), .o1(new_n274));
  oaih12aa1n02x5               g179(.a(new_n267), .b(new_n274), .c(new_n264), .o1(new_n275));
  tech160nm_fiaoi012aa1n02p5x5 g180(.a(new_n272), .b(new_n275), .c(new_n270), .o1(new_n276));
  norp02aa1n03x5               g181(.a(new_n276), .b(new_n273), .o1(\s[26] ));
  nor042aa1n03x5               g182(.a(new_n272), .b(new_n266), .o1(new_n278));
  inv000aa1n02x5               g183(.a(new_n278), .o1(new_n279));
  nona32aa1n09x5               g184(.a(new_n204), .b(new_n279), .c(new_n260), .d(new_n222), .out0(new_n280));
  oao003aa1n02x5               g185(.a(\a[26] ), .b(\b[25] ), .c(new_n270), .carry(new_n281));
  aobi12aa1n12x5               g186(.a(new_n281), .b(new_n264), .c(new_n278), .out0(new_n282));
  norp02aa1n02x5               g187(.a(\b[26] ), .b(\a[27] ), .o1(new_n283));
  nanp02aa1n02x5               g188(.a(\b[26] ), .b(\a[27] ), .o1(new_n284));
  norb02aa1n02x5               g189(.a(new_n284), .b(new_n283), .out0(new_n285));
  xnbna2aa1n03x5               g190(.a(new_n285), .b(new_n282), .c(new_n280), .out0(\s[27] ));
  inv000aa1n06x5               g191(.a(new_n283), .o1(new_n287));
  xnrc02aa1n02x5               g192(.a(\b[27] ), .b(\a[28] ), .out0(new_n288));
  nanp03aa1n02x5               g193(.a(new_n239), .b(new_n259), .c(new_n278), .o1(new_n289));
  aoi112aa1n03x5               g194(.a(new_n289), .b(new_n222), .c(new_n187), .d(new_n192), .o1(new_n290));
  aoai13aa1n04x5               g195(.a(new_n281), .b(new_n279), .c(new_n262), .d(new_n263), .o1(new_n291));
  oai012aa1n03x5               g196(.a(new_n284), .b(new_n290), .c(new_n291), .o1(new_n292));
  tech160nm_fiaoi012aa1n02p5x5 g197(.a(new_n288), .b(new_n292), .c(new_n287), .o1(new_n293));
  aoi022aa1n02x7               g198(.a(new_n282), .b(new_n280), .c(\a[27] ), .d(\b[26] ), .o1(new_n294));
  nano22aa1n03x5               g199(.a(new_n294), .b(new_n287), .c(new_n288), .out0(new_n295));
  norp02aa1n03x5               g200(.a(new_n293), .b(new_n295), .o1(\s[28] ));
  nano22aa1n02x4               g201(.a(new_n288), .b(new_n287), .c(new_n284), .out0(new_n297));
  oai012aa1n03x5               g202(.a(new_n297), .b(new_n290), .c(new_n291), .o1(new_n298));
  oao003aa1n02x5               g203(.a(\a[28] ), .b(\b[27] ), .c(new_n287), .carry(new_n299));
  xnrc02aa1n02x5               g204(.a(\b[28] ), .b(\a[29] ), .out0(new_n300));
  tech160nm_fiaoi012aa1n02p5x5 g205(.a(new_n300), .b(new_n298), .c(new_n299), .o1(new_n301));
  aobi12aa1n06x5               g206(.a(new_n297), .b(new_n282), .c(new_n280), .out0(new_n302));
  nano22aa1n03x5               g207(.a(new_n302), .b(new_n299), .c(new_n300), .out0(new_n303));
  norp02aa1n03x5               g208(.a(new_n301), .b(new_n303), .o1(\s[29] ));
  xorb03aa1n02x5               g209(.a(new_n111), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g210(.a(new_n285), .b(new_n300), .c(new_n288), .out0(new_n306));
  oai012aa1n03x5               g211(.a(new_n306), .b(new_n290), .c(new_n291), .o1(new_n307));
  oao003aa1n02x5               g212(.a(\a[29] ), .b(\b[28] ), .c(new_n299), .carry(new_n308));
  xnrc02aa1n02x5               g213(.a(\b[29] ), .b(\a[30] ), .out0(new_n309));
  tech160nm_fiaoi012aa1n02p5x5 g214(.a(new_n309), .b(new_n307), .c(new_n308), .o1(new_n310));
  aobi12aa1n06x5               g215(.a(new_n306), .b(new_n282), .c(new_n280), .out0(new_n311));
  nano22aa1n03x5               g216(.a(new_n311), .b(new_n308), .c(new_n309), .out0(new_n312));
  norp02aa1n03x5               g217(.a(new_n310), .b(new_n312), .o1(\s[30] ));
  norb03aa1n02x5               g218(.a(new_n297), .b(new_n309), .c(new_n300), .out0(new_n314));
  aobi12aa1n06x5               g219(.a(new_n314), .b(new_n282), .c(new_n280), .out0(new_n315));
  oao003aa1n02x5               g220(.a(\a[30] ), .b(\b[29] ), .c(new_n308), .carry(new_n316));
  xnrc02aa1n02x5               g221(.a(\b[30] ), .b(\a[31] ), .out0(new_n317));
  nano22aa1n03x5               g222(.a(new_n315), .b(new_n316), .c(new_n317), .out0(new_n318));
  oai012aa1n03x5               g223(.a(new_n314), .b(new_n290), .c(new_n291), .o1(new_n319));
  tech160nm_fiaoi012aa1n02p5x5 g224(.a(new_n317), .b(new_n319), .c(new_n316), .o1(new_n320));
  norp02aa1n03x5               g225(.a(new_n320), .b(new_n318), .o1(\s[31] ));
  xobna2aa1n03x5               g226(.a(new_n109), .b(new_n112), .c(new_n110), .out0(\s[3] ));
  nanp02aa1n02x5               g227(.a(\b[3] ), .b(\a[4] ), .o1(new_n323));
  nanp02aa1n02x5               g228(.a(new_n117), .b(new_n323), .o1(new_n324));
  aboi22aa1n03x5               g229(.a(new_n115), .b(new_n323), .c(new_n113), .d(new_n114), .out0(new_n325));
  aoai13aa1n02x5               g230(.a(new_n325), .b(new_n109), .c(new_n112), .d(new_n110), .o1(new_n326));
  oa0012aa1n02x5               g231(.a(new_n326), .b(new_n324), .c(new_n115), .o(\s[4] ));
  xnrb03aa1n02x5               g232(.a(new_n324), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oaoi03aa1n02x5               g233(.a(\a[5] ), .b(\b[4] ), .c(new_n324), .o1(new_n329));
  xorb03aa1n02x5               g234(.a(new_n329), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  oaoi13aa1n02x5               g235(.a(new_n119), .b(new_n103), .c(new_n329), .d(new_n105), .o1(new_n331));
  oai112aa1n02x5               g236(.a(new_n103), .b(new_n119), .c(new_n329), .d(new_n105), .o1(new_n332));
  norb02aa1n02x5               g237(.a(new_n332), .b(new_n331), .out0(\s[7] ));
  oai012aa1n02x5               g238(.a(new_n332), .b(\b[6] ), .c(\a[7] ), .o1(new_n334));
  xorb03aa1n02x5               g239(.a(new_n334), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xnbna2aa1n03x5               g240(.a(new_n135), .b(new_n124), .c(new_n98), .out0(\s[9] ));
endmodule


