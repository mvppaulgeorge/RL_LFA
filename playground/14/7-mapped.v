// Benchmark "adder" written by ABC on Wed Jul 17 19:09:44 2024

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
    new_n132, new_n133, new_n134, new_n135, new_n137, new_n138, new_n139,
    new_n140, new_n141, new_n142, new_n143, new_n144, new_n146, new_n147,
    new_n148, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n162, new_n163,
    new_n164, new_n166, new_n167, new_n168, new_n169, new_n170, new_n171,
    new_n172, new_n174, new_n175, new_n176, new_n177, new_n178, new_n179,
    new_n180, new_n181, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n194, new_n195,
    new_n196, new_n198, new_n199, new_n200, new_n201, new_n202, new_n203,
    new_n204, new_n205, new_n206, new_n209, new_n210, new_n211, new_n212,
    new_n213, new_n214, new_n215, new_n216, new_n217, new_n218, new_n219,
    new_n221, new_n222, new_n223, new_n224, new_n225, new_n226, new_n228,
    new_n229, new_n230, new_n231, new_n232, new_n233, new_n234, new_n235,
    new_n236, new_n238, new_n239, new_n240, new_n241, new_n242, new_n243,
    new_n244, new_n245, new_n246, new_n248, new_n249, new_n250, new_n251,
    new_n252, new_n253, new_n254, new_n255, new_n256, new_n257, new_n258,
    new_n260, new_n261, new_n262, new_n263, new_n264, new_n265, new_n266,
    new_n268, new_n269, new_n270, new_n271, new_n272, new_n273, new_n274,
    new_n275, new_n276, new_n278, new_n279, new_n280, new_n281, new_n282,
    new_n283, new_n284, new_n285, new_n286, new_n287, new_n288, new_n289,
    new_n290, new_n291, new_n292, new_n293, new_n294, new_n295, new_n297,
    new_n298, new_n299, new_n300, new_n301, new_n302, new_n303, new_n304,
    new_n305, new_n306, new_n307, new_n309, new_n310, new_n311, new_n312,
    new_n313, new_n314, new_n315, new_n316, new_n319, new_n320, new_n321,
    new_n322, new_n323, new_n324, new_n325, new_n326, new_n328, new_n329,
    new_n330, new_n331, new_n332, new_n333, new_n334, new_n335, new_n338,
    new_n339, new_n342, new_n344, new_n346;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor002aa1d32x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(new_n97), .o1(new_n98));
  and002aa1n24x5               g003(.a(\b[3] ), .b(\a[4] ), .o(new_n99));
  inv040aa1d32x5               g004(.a(\a[3] ), .o1(new_n100));
  inv040aa1d28x5               g005(.a(\b[2] ), .o1(new_n101));
  nor002aa1n02x5               g006(.a(\b[3] ), .b(\a[4] ), .o1(new_n102));
  aoi112aa1n06x5               g007(.a(new_n99), .b(new_n102), .c(new_n100), .d(new_n101), .o1(new_n103));
  nanp02aa1n02x5               g008(.a(new_n101), .b(new_n100), .o1(new_n104));
  nanp02aa1n02x5               g009(.a(\b[2] ), .b(\a[3] ), .o1(new_n105));
  nand02aa1n02x5               g010(.a(new_n104), .b(new_n105), .o1(new_n106));
  nor042aa1n03x5               g011(.a(\b[1] ), .b(\a[2] ), .o1(new_n107));
  nand02aa1n06x5               g012(.a(\b[0] ), .b(\a[1] ), .o1(new_n108));
  nand42aa1n02x5               g013(.a(\b[1] ), .b(\a[2] ), .o1(new_n109));
  oai012aa1n09x5               g014(.a(new_n109), .b(new_n107), .c(new_n108), .o1(new_n110));
  oaoi13aa1n12x5               g015(.a(new_n99), .b(new_n103), .c(new_n110), .d(new_n106), .o1(new_n111));
  nanp02aa1n02x5               g016(.a(\b[4] ), .b(\a[5] ), .o1(new_n112));
  inv040aa1d28x5               g017(.a(\a[6] ), .o1(new_n113));
  inv000aa1d42x5               g018(.a(\b[5] ), .o1(new_n114));
  nand42aa1n04x5               g019(.a(new_n114), .b(new_n113), .o1(new_n115));
  tech160nm_finand02aa1n05x5   g020(.a(\b[5] ), .b(\a[6] ), .o1(new_n116));
  oai112aa1n02x5               g021(.a(new_n115), .b(new_n116), .c(\b[4] ), .d(\a[5] ), .o1(new_n117));
  nand42aa1n04x5               g022(.a(\b[6] ), .b(\a[7] ), .o1(new_n118));
  nor002aa1d32x5               g023(.a(\b[7] ), .b(\a[8] ), .o1(new_n119));
  inv000aa1d42x5               g024(.a(new_n119), .o1(new_n120));
  nand02aa1n06x5               g025(.a(\b[7] ), .b(\a[8] ), .o1(new_n121));
  oai112aa1n02x5               g026(.a(new_n120), .b(new_n121), .c(\b[6] ), .d(\a[7] ), .o1(new_n122));
  nano23aa1n06x5               g027(.a(new_n122), .b(new_n117), .c(new_n112), .d(new_n118), .out0(new_n123));
  nor022aa1n16x5               g028(.a(\b[6] ), .b(\a[7] ), .o1(new_n124));
  nona23aa1n06x5               g029(.a(new_n121), .b(new_n118), .c(new_n124), .d(new_n119), .out0(new_n125));
  tech160nm_fioai012aa1n03p5x5 g030(.a(new_n121), .b(new_n124), .c(new_n119), .o1(new_n126));
  nor042aa1n02x5               g031(.a(\b[4] ), .b(\a[5] ), .o1(new_n127));
  tech160nm_fioaoi03aa1n03p5x5 g032(.a(new_n113), .b(new_n114), .c(new_n127), .o1(new_n128));
  oai012aa1d24x5               g033(.a(new_n126), .b(new_n125), .c(new_n128), .o1(new_n129));
  and002aa1n12x5               g034(.a(\b[8] ), .b(\a[9] ), .o(new_n130));
  norp02aa1n02x5               g035(.a(new_n130), .b(new_n97), .o1(new_n131));
  aoai13aa1n06x5               g036(.a(new_n131), .b(new_n129), .c(new_n111), .d(new_n123), .o1(new_n132));
  norp02aa1n12x5               g037(.a(\b[9] ), .b(\a[10] ), .o1(new_n133));
  nand42aa1d28x5               g038(.a(\b[9] ), .b(\a[10] ), .o1(new_n134));
  norb02aa1n02x5               g039(.a(new_n134), .b(new_n133), .out0(new_n135));
  xnbna2aa1n03x5               g040(.a(new_n135), .b(new_n132), .c(new_n98), .out0(\s[10] ));
  inv000aa1d42x5               g041(.a(new_n134), .o1(new_n137));
  nor002aa1d32x5               g042(.a(\b[10] ), .b(\a[11] ), .o1(new_n138));
  nand22aa1n12x5               g043(.a(\b[10] ), .b(\a[11] ), .o1(new_n139));
  inv000aa1n02x5               g044(.a(new_n139), .o1(new_n140));
  nona22aa1n02x4               g045(.a(new_n132), .b(new_n133), .c(new_n97), .out0(new_n141));
  nona32aa1n02x4               g046(.a(new_n141), .b(new_n140), .c(new_n138), .d(new_n137), .out0(new_n142));
  inv000aa1d42x5               g047(.a(new_n138), .o1(new_n143));
  aoi022aa1n02x5               g048(.a(new_n141), .b(new_n134), .c(new_n143), .d(new_n139), .o1(new_n144));
  norb02aa1n02x5               g049(.a(new_n142), .b(new_n144), .out0(\s[11] ));
  nor002aa1d24x5               g050(.a(\b[11] ), .b(\a[12] ), .o1(new_n146));
  nand42aa1n16x5               g051(.a(\b[11] ), .b(\a[12] ), .o1(new_n147));
  norb02aa1n02x5               g052(.a(new_n147), .b(new_n146), .out0(new_n148));
  xnbna2aa1n03x5               g053(.a(new_n148), .b(new_n142), .c(new_n143), .out0(\s[12] ));
  norb03aa1d15x5               g054(.a(new_n134), .b(new_n97), .c(new_n133), .out0(new_n150));
  norb03aa1d15x5               g055(.a(new_n147), .b(new_n138), .c(new_n146), .out0(new_n151));
  nona23aa1d16x5               g056(.a(new_n150), .b(new_n151), .c(new_n140), .d(new_n130), .out0(new_n152));
  inv000aa1d42x5               g057(.a(new_n152), .o1(new_n153));
  aoai13aa1n06x5               g058(.a(new_n153), .b(new_n129), .c(new_n111), .d(new_n123), .o1(new_n154));
  nona23aa1n09x5               g059(.a(new_n147), .b(new_n139), .c(new_n138), .d(new_n146), .out0(new_n155));
  oaih12aa1n12x5               g060(.a(new_n147), .b(new_n146), .c(new_n138), .o1(new_n156));
  oai012aa1n12x5               g061(.a(new_n134), .b(new_n133), .c(new_n97), .o1(new_n157));
  oai012aa1d24x5               g062(.a(new_n156), .b(new_n155), .c(new_n157), .o1(new_n158));
  inv000aa1d42x5               g063(.a(new_n158), .o1(new_n159));
  nanp02aa1n02x5               g064(.a(new_n154), .b(new_n159), .o1(new_n160));
  xorb03aa1n02x5               g065(.a(new_n160), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  inv000aa1d42x5               g066(.a(\a[13] ), .o1(new_n162));
  inv000aa1d42x5               g067(.a(\b[12] ), .o1(new_n163));
  oaoi03aa1n02x5               g068(.a(new_n162), .b(new_n163), .c(new_n160), .o1(new_n164));
  xnrb03aa1n02x5               g069(.a(new_n164), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  nor022aa1n06x5               g070(.a(\b[13] ), .b(\a[14] ), .o1(new_n166));
  nand42aa1n03x5               g071(.a(\b[13] ), .b(\a[14] ), .o1(new_n167));
  aoai13aa1n03x5               g072(.a(new_n167), .b(new_n166), .c(new_n162), .d(new_n163), .o1(new_n168));
  nor002aa1n02x5               g073(.a(\b[12] ), .b(\a[13] ), .o1(new_n169));
  nand42aa1n03x5               g074(.a(\b[12] ), .b(\a[13] ), .o1(new_n170));
  nona23aa1n09x5               g075(.a(new_n167), .b(new_n170), .c(new_n169), .d(new_n166), .out0(new_n171));
  aoai13aa1n06x5               g076(.a(new_n168), .b(new_n171), .c(new_n154), .d(new_n159), .o1(new_n172));
  xorb03aa1n02x5               g077(.a(new_n172), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  norp02aa1n04x5               g078(.a(\b[14] ), .b(\a[15] ), .o1(new_n174));
  tech160nm_finand02aa1n03p5x5 g079(.a(\b[14] ), .b(\a[15] ), .o1(new_n175));
  norp02aa1n04x5               g080(.a(\b[15] ), .b(\a[16] ), .o1(new_n176));
  nand42aa1n03x5               g081(.a(\b[15] ), .b(\a[16] ), .o1(new_n177));
  nanb02aa1n02x5               g082(.a(new_n176), .b(new_n177), .out0(new_n178));
  inv000aa1d42x5               g083(.a(new_n178), .o1(new_n179));
  aoi112aa1n02x5               g084(.a(new_n179), .b(new_n174), .c(new_n172), .d(new_n175), .o1(new_n180));
  aoai13aa1n03x5               g085(.a(new_n179), .b(new_n174), .c(new_n172), .d(new_n175), .o1(new_n181));
  norb02aa1n02x5               g086(.a(new_n181), .b(new_n180), .out0(\s[16] ));
  nano23aa1n03x7               g087(.a(new_n169), .b(new_n166), .c(new_n167), .d(new_n170), .out0(new_n183));
  nano23aa1n03x7               g088(.a(new_n174), .b(new_n176), .c(new_n177), .d(new_n175), .out0(new_n184));
  nano22aa1n12x5               g089(.a(new_n152), .b(new_n183), .c(new_n184), .out0(new_n185));
  aoai13aa1n12x5               g090(.a(new_n185), .b(new_n129), .c(new_n111), .d(new_n123), .o1(new_n186));
  nona23aa1n09x5               g091(.a(new_n177), .b(new_n175), .c(new_n174), .d(new_n176), .out0(new_n187));
  norp02aa1n03x5               g092(.a(new_n187), .b(new_n171), .o1(new_n188));
  oa0012aa1n02x5               g093(.a(new_n177), .b(new_n176), .c(new_n174), .o(new_n189));
  norp02aa1n02x5               g094(.a(new_n187), .b(new_n168), .o1(new_n190));
  aoi112aa1n09x5               g095(.a(new_n190), .b(new_n189), .c(new_n158), .d(new_n188), .o1(new_n191));
  nanp02aa1n06x5               g096(.a(new_n186), .b(new_n191), .o1(new_n192));
  xorb03aa1n02x5               g097(.a(new_n192), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv000aa1d42x5               g098(.a(\a[17] ), .o1(new_n194));
  inv000aa1d42x5               g099(.a(\b[16] ), .o1(new_n195));
  oaoi03aa1n03x5               g100(.a(new_n194), .b(new_n195), .c(new_n192), .o1(new_n196));
  xnrb03aa1n03x5               g101(.a(new_n196), .b(\b[17] ), .c(\a[18] ), .out0(\s[18] ));
  nanp02aa1n02x5               g102(.a(new_n195), .b(new_n194), .o1(new_n198));
  nanp02aa1n02x5               g103(.a(\b[16] ), .b(\a[17] ), .o1(new_n199));
  nor022aa1n06x5               g104(.a(\b[17] ), .b(\a[18] ), .o1(new_n200));
  nanp02aa1n04x5               g105(.a(\b[17] ), .b(\a[18] ), .o1(new_n201));
  nanb02aa1n03x5               g106(.a(new_n200), .b(new_n201), .out0(new_n202));
  nano22aa1n09x5               g107(.a(new_n202), .b(new_n198), .c(new_n199), .out0(new_n203));
  inv000aa1d42x5               g108(.a(new_n203), .o1(new_n204));
  aoai13aa1n12x5               g109(.a(new_n201), .b(new_n200), .c(new_n194), .d(new_n195), .o1(new_n205));
  aoai13aa1n06x5               g110(.a(new_n205), .b(new_n204), .c(new_n186), .d(new_n191), .o1(new_n206));
  xorb03aa1n02x5               g111(.a(new_n206), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g112(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor042aa1n09x5               g113(.a(\b[18] ), .b(\a[19] ), .o1(new_n209));
  nand42aa1n02x5               g114(.a(\b[18] ), .b(\a[19] ), .o1(new_n210));
  nor042aa1n02x5               g115(.a(\b[19] ), .b(\a[20] ), .o1(new_n211));
  nanp02aa1n02x5               g116(.a(\b[19] ), .b(\a[20] ), .o1(new_n212));
  nanb02aa1n02x5               g117(.a(new_n211), .b(new_n212), .out0(new_n213));
  inv000aa1d42x5               g118(.a(new_n213), .o1(new_n214));
  aoi112aa1n03x4               g119(.a(new_n209), .b(new_n214), .c(new_n206), .d(new_n210), .o1(new_n215));
  inv000aa1d42x5               g120(.a(new_n209), .o1(new_n216));
  norb02aa1n02x5               g121(.a(new_n210), .b(new_n209), .out0(new_n217));
  nanp02aa1n03x5               g122(.a(new_n206), .b(new_n217), .o1(new_n218));
  tech160nm_fiaoi012aa1n05x5   g123(.a(new_n213), .b(new_n218), .c(new_n216), .o1(new_n219));
  norp02aa1n03x5               g124(.a(new_n219), .b(new_n215), .o1(\s[20] ));
  nano23aa1n09x5               g125(.a(new_n209), .b(new_n211), .c(new_n212), .d(new_n210), .out0(new_n221));
  nanp02aa1n02x5               g126(.a(new_n203), .b(new_n221), .o1(new_n222));
  inv000aa1n02x5               g127(.a(new_n205), .o1(new_n223));
  oai012aa1n02x5               g128(.a(new_n212), .b(new_n211), .c(new_n209), .o1(new_n224));
  aobi12aa1n03x7               g129(.a(new_n224), .b(new_n221), .c(new_n223), .out0(new_n225));
  aoai13aa1n06x5               g130(.a(new_n225), .b(new_n222), .c(new_n186), .d(new_n191), .o1(new_n226));
  xorb03aa1n02x5               g131(.a(new_n226), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor042aa1n06x5               g132(.a(\b[20] ), .b(\a[21] ), .o1(new_n228));
  nanp02aa1n02x5               g133(.a(\b[20] ), .b(\a[21] ), .o1(new_n229));
  xorc02aa1n02x5               g134(.a(\a[22] ), .b(\b[21] ), .out0(new_n230));
  aoi112aa1n03x4               g135(.a(new_n228), .b(new_n230), .c(new_n226), .d(new_n229), .o1(new_n231));
  inv000aa1d42x5               g136(.a(new_n228), .o1(new_n232));
  norb02aa1n02x5               g137(.a(new_n229), .b(new_n228), .out0(new_n233));
  nanp02aa1n03x5               g138(.a(new_n226), .b(new_n233), .o1(new_n234));
  inv000aa1d42x5               g139(.a(new_n230), .o1(new_n235));
  tech160nm_fiaoi012aa1n03p5x5 g140(.a(new_n235), .b(new_n234), .c(new_n232), .o1(new_n236));
  nor042aa1n03x5               g141(.a(new_n236), .b(new_n231), .o1(\s[22] ));
  norp02aa1n02x5               g142(.a(\b[21] ), .b(\a[22] ), .o1(new_n238));
  nanp02aa1n02x5               g143(.a(\b[21] ), .b(\a[22] ), .o1(new_n239));
  nano23aa1n09x5               g144(.a(new_n228), .b(new_n238), .c(new_n239), .d(new_n229), .out0(new_n240));
  nanp03aa1n03x5               g145(.a(new_n203), .b(new_n221), .c(new_n240), .o1(new_n241));
  nona23aa1n02x4               g146(.a(new_n212), .b(new_n210), .c(new_n209), .d(new_n211), .out0(new_n242));
  tech160nm_fioai012aa1n03p5x5 g147(.a(new_n224), .b(new_n242), .c(new_n205), .o1(new_n243));
  tech160nm_fioaoi03aa1n02p5x5 g148(.a(\a[22] ), .b(\b[21] ), .c(new_n232), .o1(new_n244));
  aoi012aa1n02x5               g149(.a(new_n244), .b(new_n243), .c(new_n240), .o1(new_n245));
  aoai13aa1n06x5               g150(.a(new_n245), .b(new_n241), .c(new_n186), .d(new_n191), .o1(new_n246));
  xorb03aa1n02x5               g151(.a(new_n246), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  nor022aa1n12x5               g152(.a(\b[22] ), .b(\a[23] ), .o1(new_n248));
  nanp02aa1n02x5               g153(.a(\b[22] ), .b(\a[23] ), .o1(new_n249));
  nor002aa1n02x5               g154(.a(\b[23] ), .b(\a[24] ), .o1(new_n250));
  nanp02aa1n02x5               g155(.a(\b[23] ), .b(\a[24] ), .o1(new_n251));
  norb02aa1n02x5               g156(.a(new_n251), .b(new_n250), .out0(new_n252));
  aoi112aa1n03x4               g157(.a(new_n248), .b(new_n252), .c(new_n246), .d(new_n249), .o1(new_n253));
  inv000aa1d42x5               g158(.a(new_n248), .o1(new_n254));
  norb02aa1n02x5               g159(.a(new_n249), .b(new_n248), .out0(new_n255));
  nand02aa1d04x5               g160(.a(new_n246), .b(new_n255), .o1(new_n256));
  inv000aa1d42x5               g161(.a(new_n252), .o1(new_n257));
  tech160nm_fiaoi012aa1n03p5x5 g162(.a(new_n257), .b(new_n256), .c(new_n254), .o1(new_n258));
  nor042aa1n03x5               g163(.a(new_n258), .b(new_n253), .o1(\s[24] ));
  nano23aa1n06x5               g164(.a(new_n248), .b(new_n250), .c(new_n251), .d(new_n249), .out0(new_n260));
  nanb03aa1n02x5               g165(.a(new_n222), .b(new_n260), .c(new_n240), .out0(new_n261));
  nona22aa1n02x4               g166(.a(new_n251), .b(new_n250), .c(new_n248), .out0(new_n262));
  aoi022aa1n06x5               g167(.a(new_n260), .b(new_n244), .c(new_n262), .d(new_n251), .o1(new_n263));
  inv020aa1n03x5               g168(.a(new_n263), .o1(new_n264));
  aoi013aa1n02x4               g169(.a(new_n264), .b(new_n243), .c(new_n240), .d(new_n260), .o1(new_n265));
  aoai13aa1n06x5               g170(.a(new_n265), .b(new_n261), .c(new_n186), .d(new_n191), .o1(new_n266));
  xorb03aa1n02x5               g171(.a(new_n266), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  nor042aa1n03x5               g172(.a(\b[24] ), .b(\a[25] ), .o1(new_n268));
  nanp02aa1n02x5               g173(.a(\b[24] ), .b(\a[25] ), .o1(new_n269));
  xnrc02aa1n12x5               g174(.a(\b[25] ), .b(\a[26] ), .out0(new_n270));
  inv000aa1d42x5               g175(.a(new_n270), .o1(new_n271));
  aoi112aa1n03x4               g176(.a(new_n268), .b(new_n271), .c(new_n266), .d(new_n269), .o1(new_n272));
  inv000aa1n06x5               g177(.a(new_n268), .o1(new_n273));
  norb02aa1n02x5               g178(.a(new_n269), .b(new_n268), .out0(new_n274));
  nand02aa1n02x5               g179(.a(new_n266), .b(new_n274), .o1(new_n275));
  aoi012aa1n06x5               g180(.a(new_n270), .b(new_n275), .c(new_n273), .o1(new_n276));
  norp02aa1n02x5               g181(.a(new_n276), .b(new_n272), .o1(\s[26] ));
  inv000aa1d42x5               g182(.a(new_n99), .o1(new_n278));
  tech160nm_fioai012aa1n04x5   g183(.a(new_n103), .b(new_n110), .c(new_n106), .o1(new_n279));
  nanp02aa1n02x5               g184(.a(new_n279), .b(new_n278), .o1(new_n280));
  nano23aa1n02x4               g185(.a(new_n124), .b(new_n119), .c(new_n118), .d(new_n121), .out0(new_n281));
  nanb03aa1n02x5               g186(.a(new_n117), .b(new_n281), .c(new_n112), .out0(new_n282));
  inv000aa1n02x5               g187(.a(new_n129), .o1(new_n283));
  tech160nm_fioai012aa1n05x5   g188(.a(new_n283), .b(new_n280), .c(new_n282), .o1(new_n284));
  nanp02aa1n02x5               g189(.a(new_n158), .b(new_n188), .o1(new_n285));
  nona22aa1n02x4               g190(.a(new_n285), .b(new_n190), .c(new_n189), .out0(new_n286));
  nano22aa1n09x5               g191(.a(new_n270), .b(new_n273), .c(new_n269), .out0(new_n287));
  nano22aa1n03x7               g192(.a(new_n241), .b(new_n260), .c(new_n287), .out0(new_n288));
  aoai13aa1n06x5               g193(.a(new_n288), .b(new_n286), .c(new_n284), .d(new_n185), .o1(new_n289));
  nano22aa1n03x7               g194(.a(new_n225), .b(new_n240), .c(new_n260), .out0(new_n290));
  oaoi03aa1n12x5               g195(.a(\a[26] ), .b(\b[25] ), .c(new_n273), .o1(new_n291));
  oaoi13aa1n09x5               g196(.a(new_n291), .b(new_n287), .c(new_n290), .d(new_n264), .o1(new_n292));
  nor042aa1n06x5               g197(.a(\b[26] ), .b(\a[27] ), .o1(new_n293));
  nanp02aa1n02x5               g198(.a(\b[26] ), .b(\a[27] ), .o1(new_n294));
  norb02aa1d27x5               g199(.a(new_n294), .b(new_n293), .out0(new_n295));
  xnbna2aa1n03x5               g200(.a(new_n295), .b(new_n289), .c(new_n292), .out0(\s[27] ));
  inv030aa1n02x5               g201(.a(new_n293), .o1(new_n297));
  inv000aa1d42x5               g202(.a(new_n295), .o1(new_n298));
  tech160nm_fiaoi012aa1n02p5x5 g203(.a(new_n298), .b(new_n289), .c(new_n292), .o1(new_n299));
  xnrc02aa1n06x5               g204(.a(\b[27] ), .b(\a[28] ), .out0(new_n300));
  nano22aa1n02x4               g205(.a(new_n299), .b(new_n297), .c(new_n300), .out0(new_n301));
  nanp03aa1n02x5               g206(.a(new_n243), .b(new_n240), .c(new_n260), .o1(new_n302));
  inv000aa1d42x5               g207(.a(new_n287), .o1(new_n303));
  inv000aa1d42x5               g208(.a(new_n291), .o1(new_n304));
  aoai13aa1n06x5               g209(.a(new_n304), .b(new_n303), .c(new_n302), .d(new_n263), .o1(new_n305));
  aoai13aa1n03x5               g210(.a(new_n295), .b(new_n305), .c(new_n192), .d(new_n288), .o1(new_n306));
  tech160nm_fiaoi012aa1n02p5x5 g211(.a(new_n300), .b(new_n306), .c(new_n297), .o1(new_n307));
  norp02aa1n03x5               g212(.a(new_n307), .b(new_n301), .o1(\s[28] ));
  xnrc02aa1n12x5               g213(.a(\b[28] ), .b(\a[29] ), .out0(new_n309));
  nano22aa1n06x5               g214(.a(new_n300), .b(new_n297), .c(new_n294), .out0(new_n310));
  aoai13aa1n03x5               g215(.a(new_n310), .b(new_n305), .c(new_n192), .d(new_n288), .o1(new_n311));
  oao003aa1n02x5               g216(.a(\a[28] ), .b(\b[27] ), .c(new_n297), .carry(new_n312));
  tech160nm_fiaoi012aa1n02p5x5 g217(.a(new_n309), .b(new_n311), .c(new_n312), .o1(new_n313));
  inv000aa1d42x5               g218(.a(new_n310), .o1(new_n314));
  tech160nm_fiaoi012aa1n02p5x5 g219(.a(new_n314), .b(new_n289), .c(new_n292), .o1(new_n315));
  nano22aa1n02x4               g220(.a(new_n315), .b(new_n309), .c(new_n312), .out0(new_n316));
  norp02aa1n03x5               g221(.a(new_n313), .b(new_n316), .o1(\s[29] ));
  xorb03aa1n02x5               g222(.a(new_n108), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano23aa1d15x5               g223(.a(new_n309), .b(new_n300), .c(new_n294), .d(new_n297), .out0(new_n319));
  aoai13aa1n03x5               g224(.a(new_n319), .b(new_n305), .c(new_n192), .d(new_n288), .o1(new_n320));
  oao003aa1n02x5               g225(.a(\a[29] ), .b(\b[28] ), .c(new_n312), .carry(new_n321));
  xnrc02aa1n02x5               g226(.a(\b[29] ), .b(\a[30] ), .out0(new_n322));
  aoi012aa1n03x5               g227(.a(new_n322), .b(new_n320), .c(new_n321), .o1(new_n323));
  inv030aa1n02x5               g228(.a(new_n319), .o1(new_n324));
  aoi012aa1n06x5               g229(.a(new_n324), .b(new_n289), .c(new_n292), .o1(new_n325));
  nano22aa1n02x5               g230(.a(new_n325), .b(new_n321), .c(new_n322), .out0(new_n326));
  norp02aa1n03x5               g231(.a(new_n323), .b(new_n326), .o1(\s[30] ));
  xnrc02aa1n02x5               g232(.a(\b[30] ), .b(\a[31] ), .out0(new_n328));
  norb03aa1d15x5               g233(.a(new_n310), .b(new_n309), .c(new_n322), .out0(new_n329));
  inv000aa1d42x5               g234(.a(new_n329), .o1(new_n330));
  tech160nm_fiaoi012aa1n02p5x5 g235(.a(new_n330), .b(new_n289), .c(new_n292), .o1(new_n331));
  oao003aa1n02x5               g236(.a(\a[30] ), .b(\b[29] ), .c(new_n321), .carry(new_n332));
  nano22aa1n02x4               g237(.a(new_n331), .b(new_n328), .c(new_n332), .out0(new_n333));
  aoai13aa1n03x5               g238(.a(new_n329), .b(new_n305), .c(new_n192), .d(new_n288), .o1(new_n334));
  tech160nm_fiaoi012aa1n05x5   g239(.a(new_n328), .b(new_n334), .c(new_n332), .o1(new_n335));
  nor002aa1n02x5               g240(.a(new_n335), .b(new_n333), .o1(\s[31] ));
  xnbna2aa1n03x5               g241(.a(new_n110), .b(new_n104), .c(new_n105), .out0(\s[3] ));
  xnrc02aa1n02x5               g242(.a(\b[3] ), .b(\a[4] ), .out0(new_n338));
  oaoi03aa1n02x5               g243(.a(\a[3] ), .b(\b[2] ), .c(new_n110), .o1(new_n339));
  aob012aa1n02x5               g244(.a(new_n279), .b(new_n339), .c(new_n338), .out0(\s[4] ));
  xorb03aa1n02x5               g245(.a(new_n111), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  aoi013aa1n06x4               g246(.a(new_n127), .b(new_n279), .c(new_n112), .d(new_n278), .o1(new_n342));
  xnbna2aa1n03x5               g247(.a(new_n342), .b(new_n115), .c(new_n116), .out0(\s[6] ));
  oaoi03aa1n02x5               g248(.a(\a[6] ), .b(\b[5] ), .c(new_n342), .o1(new_n344));
  xorb03aa1n02x5               g249(.a(new_n344), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  tech160nm_fiaoi012aa1n05x5   g250(.a(new_n124), .b(new_n344), .c(new_n118), .o1(new_n346));
  xnbna2aa1n03x5               g251(.a(new_n346), .b(new_n120), .c(new_n121), .out0(\s[8] ));
  xorb03aa1n02x5               g252(.a(new_n284), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


