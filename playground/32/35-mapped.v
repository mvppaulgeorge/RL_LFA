// Benchmark "adder" written by ABC on Thu Jul 18 04:40:24 2024

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
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n138, new_n139,
    new_n140, new_n142, new_n143, new_n144, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n158, new_n159, new_n161, new_n162, new_n163, new_n164,
    new_n165, new_n166, new_n167, new_n168, new_n169, new_n170, new_n171,
    new_n173, new_n174, new_n175, new_n176, new_n177, new_n178, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n187, new_n188,
    new_n189, new_n190, new_n191, new_n193, new_n194, new_n195, new_n196,
    new_n197, new_n198, new_n201, new_n202, new_n203, new_n204, new_n205,
    new_n206, new_n207, new_n209, new_n210, new_n211, new_n212, new_n213,
    new_n214, new_n215, new_n216, new_n217, new_n218, new_n219, new_n220,
    new_n221, new_n222, new_n223, new_n225, new_n226, new_n227, new_n228,
    new_n230, new_n231, new_n232, new_n233, new_n234, new_n235, new_n236,
    new_n237, new_n239, new_n240, new_n241, new_n242, new_n243, new_n245,
    new_n246, new_n247, new_n248, new_n249, new_n250, new_n251, new_n252,
    new_n253, new_n254, new_n255, new_n256, new_n258, new_n259, new_n260,
    new_n261, new_n262, new_n263, new_n265, new_n266, new_n267, new_n268,
    new_n269, new_n270, new_n271, new_n272, new_n273, new_n275, new_n276,
    new_n277, new_n278, new_n279, new_n280, new_n281, new_n282, new_n284,
    new_n285, new_n286, new_n287, new_n288, new_n289, new_n290, new_n291,
    new_n292, new_n294, new_n296, new_n297, new_n298, new_n299, new_n300,
    new_n301, new_n302, new_n304, new_n305, new_n306, new_n307, new_n308,
    new_n309, new_n310, new_n311, new_n312, new_n314, new_n315, new_n317,
    new_n318, new_n319, new_n320, new_n321, new_n323, new_n325, new_n327,
    new_n328, new_n329, new_n331;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  inv040aa1d28x5               g001(.a(\a[9] ), .o1(new_n97));
  inv040aa1d28x5               g002(.a(\b[8] ), .o1(new_n98));
  nor042aa1n04x5               g003(.a(\b[2] ), .b(\a[3] ), .o1(new_n99));
  nand02aa1d08x5               g004(.a(\b[2] ), .b(\a[3] ), .o1(new_n100));
  norb02aa1n12x5               g005(.a(new_n100), .b(new_n99), .out0(new_n101));
  nor042aa1n04x5               g006(.a(\b[1] ), .b(\a[2] ), .o1(new_n102));
  aoi022aa1d24x5               g007(.a(\b[1] ), .b(\a[2] ), .c(\a[1] ), .d(\b[0] ), .o1(new_n103));
  oaih22aa1n06x5               g008(.a(\a[3] ), .b(\b[2] ), .c(\b[3] ), .d(\a[4] ), .o1(new_n104));
  oaoi13aa1n04x5               g009(.a(new_n104), .b(new_n101), .c(new_n103), .d(new_n102), .o1(new_n105));
  nanp02aa1n09x5               g010(.a(\b[3] ), .b(\a[4] ), .o1(new_n106));
  nand42aa1n16x5               g011(.a(\b[5] ), .b(\a[6] ), .o1(new_n107));
  oai112aa1n06x5               g012(.a(new_n106), .b(new_n107), .c(\b[4] ), .d(\a[5] ), .o1(new_n108));
  nor042aa1d18x5               g013(.a(\b[6] ), .b(\a[7] ), .o1(new_n109));
  nand02aa1n04x5               g014(.a(\b[6] ), .b(\a[7] ), .o1(new_n110));
  nanb02aa1n06x5               g015(.a(new_n109), .b(new_n110), .out0(new_n111));
  norp02aa1n24x5               g016(.a(\b[7] ), .b(\a[8] ), .o1(new_n112));
  nand02aa1n06x5               g017(.a(\b[7] ), .b(\a[8] ), .o1(new_n113));
  nand42aa1n02x5               g018(.a(\b[4] ), .b(\a[5] ), .o1(new_n114));
  norp02aa1n24x5               g019(.a(\b[5] ), .b(\a[6] ), .o1(new_n115));
  nano23aa1n02x5               g020(.a(new_n115), .b(new_n112), .c(new_n113), .d(new_n114), .out0(new_n116));
  nona22aa1n03x5               g021(.a(new_n116), .b(new_n111), .c(new_n108), .out0(new_n117));
  nano22aa1n03x7               g022(.a(new_n109), .b(new_n110), .c(new_n113), .out0(new_n118));
  nor002aa1n03x5               g023(.a(\b[4] ), .b(\a[5] ), .o1(new_n119));
  tech160nm_fioai012aa1n03p5x5 g024(.a(new_n107), .b(\b[7] ), .c(\a[8] ), .o1(new_n120));
  oab012aa1n06x5               g025(.a(new_n120), .b(new_n119), .c(new_n115), .out0(new_n121));
  aoi012aa1n02x7               g026(.a(new_n112), .b(new_n109), .c(new_n113), .o1(new_n122));
  aobi12aa1n02x7               g027(.a(new_n122), .b(new_n121), .c(new_n118), .out0(new_n123));
  tech160nm_fioai012aa1n04x5   g028(.a(new_n123), .b(new_n117), .c(new_n105), .o1(new_n124));
  oaoi03aa1n02x5               g029(.a(new_n97), .b(new_n98), .c(new_n124), .o1(new_n125));
  xnrb03aa1n02x5               g030(.a(new_n125), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  oai012aa1n06x5               g031(.a(new_n101), .b(new_n103), .c(new_n102), .o1(new_n127));
  nanb02aa1n06x5               g032(.a(new_n104), .b(new_n127), .out0(new_n128));
  nona23aa1n09x5               g033(.a(new_n114), .b(new_n113), .c(new_n115), .d(new_n112), .out0(new_n129));
  nor043aa1n02x5               g034(.a(new_n129), .b(new_n111), .c(new_n108), .o1(new_n130));
  nand42aa1n02x5               g035(.a(new_n121), .b(new_n118), .o1(new_n131));
  nand42aa1n02x5               g036(.a(new_n131), .b(new_n122), .o1(new_n132));
  aoi012aa1n12x5               g037(.a(new_n132), .b(new_n130), .c(new_n128), .o1(new_n133));
  nor002aa1d32x5               g038(.a(\b[9] ), .b(\a[10] ), .o1(new_n134));
  nanp02aa1n12x5               g039(.a(\b[9] ), .b(\a[10] ), .o1(new_n135));
  aoai13aa1n12x5               g040(.a(new_n135), .b(new_n134), .c(new_n97), .d(new_n98), .o1(new_n136));
  nor002aa1n03x5               g041(.a(\b[8] ), .b(\a[9] ), .o1(new_n137));
  nanp02aa1n02x5               g042(.a(\b[8] ), .b(\a[9] ), .o1(new_n138));
  nona23aa1n03x5               g043(.a(new_n135), .b(new_n138), .c(new_n137), .d(new_n134), .out0(new_n139));
  oai012aa1n03x5               g044(.a(new_n136), .b(new_n133), .c(new_n139), .o1(new_n140));
  xorb03aa1n02x5               g045(.a(new_n140), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  nor002aa1d32x5               g046(.a(\b[10] ), .b(\a[11] ), .o1(new_n142));
  nanp02aa1n09x5               g047(.a(\b[10] ), .b(\a[11] ), .o1(new_n143));
  aoi012aa1n02x5               g048(.a(new_n142), .b(new_n140), .c(new_n143), .o1(new_n144));
  xnrb03aa1n03x5               g049(.a(new_n144), .b(\b[11] ), .c(\a[12] ), .out0(\s[12] ));
  nor002aa1d32x5               g050(.a(\b[11] ), .b(\a[12] ), .o1(new_n146));
  nand22aa1n12x5               g051(.a(\b[11] ), .b(\a[12] ), .o1(new_n147));
  nona23aa1d18x5               g052(.a(new_n147), .b(new_n143), .c(new_n142), .d(new_n146), .out0(new_n148));
  nor042aa1n03x5               g053(.a(new_n148), .b(new_n139), .o1(new_n149));
  aoai13aa1n03x5               g054(.a(new_n149), .b(new_n132), .c(new_n130), .d(new_n128), .o1(new_n150));
  aoi012aa1n09x5               g055(.a(new_n146), .b(new_n142), .c(new_n147), .o1(new_n151));
  oai012aa1d24x5               g056(.a(new_n151), .b(new_n148), .c(new_n136), .o1(new_n152));
  inv000aa1d42x5               g057(.a(new_n152), .o1(new_n153));
  nor002aa1d32x5               g058(.a(\b[12] ), .b(\a[13] ), .o1(new_n154));
  nand42aa1d28x5               g059(.a(\b[12] ), .b(\a[13] ), .o1(new_n155));
  norb02aa1n02x5               g060(.a(new_n155), .b(new_n154), .out0(new_n156));
  xnbna2aa1n03x5               g061(.a(new_n156), .b(new_n150), .c(new_n153), .out0(\s[13] ));
  nanp02aa1n02x5               g062(.a(new_n150), .b(new_n153), .o1(new_n158));
  aoi012aa1n02x5               g063(.a(new_n154), .b(new_n158), .c(new_n155), .o1(new_n159));
  xnrb03aa1n03x5               g064(.a(new_n159), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  nor002aa1d32x5               g065(.a(\b[13] ), .b(\a[14] ), .o1(new_n161));
  nanp02aa1n24x5               g066(.a(\b[13] ), .b(\a[14] ), .o1(new_n162));
  nona23aa1d18x5               g067(.a(new_n162), .b(new_n155), .c(new_n154), .d(new_n161), .out0(new_n163));
  oa0012aa1n02x5               g068(.a(new_n162), .b(new_n161), .c(new_n154), .o(new_n164));
  inv000aa1n02x5               g069(.a(new_n164), .o1(new_n165));
  aoai13aa1n03x5               g070(.a(new_n165), .b(new_n163), .c(new_n150), .d(new_n153), .o1(new_n166));
  nor042aa1n06x5               g071(.a(\b[14] ), .b(\a[15] ), .o1(new_n167));
  nand02aa1d16x5               g072(.a(\b[14] ), .b(\a[15] ), .o1(new_n168));
  norb02aa1n06x5               g073(.a(new_n168), .b(new_n167), .out0(new_n169));
  nano23aa1n02x5               g074(.a(new_n154), .b(new_n161), .c(new_n162), .d(new_n155), .out0(new_n170));
  aoi112aa1n02x5               g075(.a(new_n169), .b(new_n164), .c(new_n158), .d(new_n170), .o1(new_n171));
  aoi012aa1n02x5               g076(.a(new_n171), .b(new_n166), .c(new_n169), .o1(\s[15] ));
  nor042aa1d18x5               g077(.a(\b[15] ), .b(\a[16] ), .o1(new_n173));
  nand02aa1d24x5               g078(.a(\b[15] ), .b(\a[16] ), .o1(new_n174));
  norb02aa1n15x5               g079(.a(new_n174), .b(new_n173), .out0(new_n175));
  inv000aa1d42x5               g080(.a(new_n175), .o1(new_n176));
  aoai13aa1n02x5               g081(.a(new_n176), .b(new_n167), .c(new_n166), .d(new_n168), .o1(new_n177));
  aoi112aa1n03x5               g082(.a(new_n167), .b(new_n176), .c(new_n166), .d(new_n168), .o1(new_n178));
  nanb02aa1n03x5               g083(.a(new_n178), .b(new_n177), .out0(\s[16] ));
  nano22aa1n09x5               g084(.a(new_n163), .b(new_n169), .c(new_n175), .out0(new_n180));
  nand02aa1n02x5               g085(.a(new_n180), .b(new_n149), .o1(new_n181));
  oai112aa1n02x5               g086(.a(new_n162), .b(new_n168), .c(new_n161), .d(new_n154), .o1(new_n182));
  aboi22aa1n06x5               g087(.a(new_n167), .b(new_n182), .c(\a[16] ), .d(\b[15] ), .out0(new_n183));
  aoi112aa1n09x5               g088(.a(new_n173), .b(new_n183), .c(new_n152), .d(new_n180), .o1(new_n184));
  oai012aa1d24x5               g089(.a(new_n184), .b(new_n133), .c(new_n181), .o1(new_n185));
  xorb03aa1n02x5               g090(.a(new_n185), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv000aa1d48x5               g091(.a(\a[17] ), .o1(new_n187));
  inv000aa1d42x5               g092(.a(\b[16] ), .o1(new_n188));
  nanp02aa1n02x5               g093(.a(new_n188), .b(new_n187), .o1(new_n189));
  oaib12aa1n06x5               g094(.a(new_n185), .b(new_n188), .c(\a[17] ), .out0(new_n190));
  xorc02aa1n02x5               g095(.a(\a[18] ), .b(\b[17] ), .out0(new_n191));
  xnbna2aa1n03x5               g096(.a(new_n191), .b(new_n190), .c(new_n189), .out0(\s[18] ));
  inv040aa1d32x5               g097(.a(\a[18] ), .o1(new_n193));
  xroi22aa1d06x4               g098(.a(new_n187), .b(\b[16] ), .c(new_n193), .d(\b[17] ), .out0(new_n194));
  oaoi03aa1n02x5               g099(.a(\a[18] ), .b(\b[17] ), .c(new_n189), .o1(new_n195));
  xorc02aa1n12x5               g100(.a(\a[19] ), .b(\b[18] ), .out0(new_n196));
  aoai13aa1n06x5               g101(.a(new_n196), .b(new_n195), .c(new_n185), .d(new_n194), .o1(new_n197));
  aoi112aa1n02x5               g102(.a(new_n196), .b(new_n195), .c(new_n185), .d(new_n194), .o1(new_n198));
  norb02aa1n03x4               g103(.a(new_n197), .b(new_n198), .out0(\s[19] ));
  xnrc02aa1n02x5               g104(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor042aa1n12x5               g105(.a(\b[18] ), .b(\a[19] ), .o1(new_n201));
  inv000aa1d42x5               g106(.a(new_n201), .o1(new_n202));
  orn002aa1n24x5               g107(.a(\a[20] ), .b(\b[19] ), .o(new_n203));
  nand02aa1d06x5               g108(.a(\b[19] ), .b(\a[20] ), .o1(new_n204));
  nanp02aa1n06x5               g109(.a(new_n203), .b(new_n204), .o1(new_n205));
  aob012aa1n03x5               g110(.a(new_n205), .b(new_n197), .c(new_n202), .out0(new_n206));
  nona22aa1n02x4               g111(.a(new_n197), .b(new_n205), .c(new_n201), .out0(new_n207));
  nanp02aa1n03x5               g112(.a(new_n206), .b(new_n207), .o1(\s[20] ));
  nanb03aa1n09x5               g113(.a(new_n205), .b(new_n194), .c(new_n196), .out0(new_n209));
  inv000aa1n02x5               g114(.a(new_n209), .o1(new_n210));
  inv000aa1d42x5               g115(.a(\a[19] ), .o1(new_n211));
  inv000aa1d42x5               g116(.a(\b[18] ), .o1(new_n212));
  oai112aa1n04x5               g117(.a(new_n203), .b(new_n204), .c(new_n212), .d(new_n211), .o1(new_n213));
  nanp02aa1n02x5               g118(.a(\b[17] ), .b(\a[18] ), .o1(new_n214));
  oaih22aa1d12x5               g119(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n215));
  oai112aa1n03x5               g120(.a(new_n215), .b(new_n214), .c(\b[18] ), .d(\a[19] ), .o1(new_n216));
  aob012aa1n03x5               g121(.a(new_n203), .b(new_n201), .c(new_n204), .out0(new_n217));
  oabi12aa1n06x5               g122(.a(new_n217), .b(new_n216), .c(new_n213), .out0(new_n218));
  norp02aa1n24x5               g123(.a(\b[20] ), .b(\a[21] ), .o1(new_n219));
  nand02aa1n02x5               g124(.a(\b[20] ), .b(\a[21] ), .o1(new_n220));
  norb02aa1n02x5               g125(.a(new_n220), .b(new_n219), .out0(new_n221));
  aoai13aa1n06x5               g126(.a(new_n221), .b(new_n218), .c(new_n185), .d(new_n210), .o1(new_n222));
  aoi112aa1n02x5               g127(.a(new_n221), .b(new_n218), .c(new_n185), .d(new_n210), .o1(new_n223));
  norb02aa1n03x4               g128(.a(new_n222), .b(new_n223), .out0(\s[21] ));
  inv040aa1n02x5               g129(.a(new_n219), .o1(new_n225));
  xnrc02aa1n12x5               g130(.a(\b[21] ), .b(\a[22] ), .out0(new_n226));
  aob012aa1n03x5               g131(.a(new_n226), .b(new_n222), .c(new_n225), .out0(new_n227));
  nona22aa1n02x4               g132(.a(new_n222), .b(new_n226), .c(new_n219), .out0(new_n228));
  nanp02aa1n03x5               g133(.a(new_n227), .b(new_n228), .o1(\s[22] ));
  nanp03aa1n02x5               g134(.a(new_n170), .b(new_n169), .c(new_n175), .o1(new_n230));
  norb02aa1n03x4               g135(.a(new_n149), .b(new_n230), .out0(new_n231));
  nand22aa1n06x5               g136(.a(new_n124), .b(new_n231), .o1(new_n232));
  nano22aa1n12x5               g137(.a(new_n226), .b(new_n225), .c(new_n220), .out0(new_n233));
  nanb02aa1n06x5               g138(.a(new_n209), .b(new_n233), .out0(new_n234));
  oaoi03aa1n02x5               g139(.a(\a[22] ), .b(\b[21] ), .c(new_n225), .o1(new_n235));
  aoi012aa1n02x5               g140(.a(new_n235), .b(new_n218), .c(new_n233), .o1(new_n236));
  aoai13aa1n06x5               g141(.a(new_n236), .b(new_n234), .c(new_n232), .d(new_n184), .o1(new_n237));
  xorb03aa1n02x5               g142(.a(new_n237), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g143(.a(\b[22] ), .b(\a[23] ), .o1(new_n239));
  xorc02aa1n06x5               g144(.a(\a[23] ), .b(\b[22] ), .out0(new_n240));
  xnrc02aa1n12x5               g145(.a(\b[23] ), .b(\a[24] ), .out0(new_n241));
  aoai13aa1n03x5               g146(.a(new_n241), .b(new_n239), .c(new_n237), .d(new_n240), .o1(new_n242));
  aoi112aa1n03x5               g147(.a(new_n239), .b(new_n241), .c(new_n237), .d(new_n240), .o1(new_n243));
  nanb02aa1n03x5               g148(.a(new_n243), .b(new_n242), .out0(\s[24] ));
  inv000aa1d42x5               g149(.a(new_n233), .o1(new_n245));
  norb02aa1n06x5               g150(.a(new_n240), .b(new_n241), .out0(new_n246));
  inv000aa1d42x5               g151(.a(new_n246), .o1(new_n247));
  nona32aa1n06x5               g152(.a(new_n185), .b(new_n247), .c(new_n245), .d(new_n209), .out0(new_n248));
  aoai13aa1n06x5               g153(.a(new_n246), .b(new_n235), .c(new_n218), .d(new_n233), .o1(new_n249));
  aoi112aa1n02x5               g154(.a(\b[22] ), .b(\a[23] ), .c(\a[24] ), .d(\b[23] ), .o1(new_n250));
  oab012aa1n02x4               g155(.a(new_n250), .b(\a[24] ), .c(\b[23] ), .out0(new_n251));
  nanp02aa1n03x5               g156(.a(new_n249), .b(new_n251), .o1(new_n252));
  inv030aa1n02x5               g157(.a(new_n252), .o1(new_n253));
  tech160nm_finand02aa1n05x5   g158(.a(new_n248), .b(new_n253), .o1(new_n254));
  xorc02aa1n12x5               g159(.a(\a[25] ), .b(\b[24] ), .out0(new_n255));
  nano22aa1n02x4               g160(.a(new_n255), .b(new_n249), .c(new_n251), .out0(new_n256));
  aoi022aa1n02x5               g161(.a(new_n254), .b(new_n255), .c(new_n248), .d(new_n256), .o1(\s[25] ));
  norp02aa1n02x5               g162(.a(\b[24] ), .b(\a[25] ), .o1(new_n258));
  xnrc02aa1n12x5               g163(.a(\b[25] ), .b(\a[26] ), .out0(new_n259));
  aoai13aa1n03x5               g164(.a(new_n259), .b(new_n258), .c(new_n254), .d(new_n255), .o1(new_n260));
  aoi112aa1n03x4               g165(.a(new_n247), .b(new_n234), .c(new_n232), .d(new_n184), .o1(new_n261));
  oai012aa1n03x5               g166(.a(new_n255), .b(new_n261), .c(new_n252), .o1(new_n262));
  nona22aa1n02x5               g167(.a(new_n262), .b(new_n259), .c(new_n258), .out0(new_n263));
  nanp02aa1n03x5               g168(.a(new_n260), .b(new_n263), .o1(\s[26] ));
  norp02aa1n02x5               g169(.a(\b[26] ), .b(\a[27] ), .o1(new_n265));
  and002aa1n02x5               g170(.a(\b[26] ), .b(\a[27] ), .o(new_n266));
  norp02aa1n03x5               g171(.a(new_n266), .b(new_n265), .o1(new_n267));
  nona23aa1n02x4               g172(.a(new_n255), .b(new_n240), .c(new_n259), .d(new_n241), .out0(new_n268));
  nona32aa1n09x5               g173(.a(new_n185), .b(new_n268), .c(new_n245), .d(new_n209), .out0(new_n269));
  norb02aa1n03x5               g174(.a(new_n255), .b(new_n259), .out0(new_n270));
  oai022aa1n02x5               g175(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n271));
  aob012aa1n02x5               g176(.a(new_n271), .b(\b[25] ), .c(\a[26] ), .out0(new_n272));
  aobi12aa1n12x5               g177(.a(new_n272), .b(new_n252), .c(new_n270), .out0(new_n273));
  xnbna2aa1n06x5               g178(.a(new_n267), .b(new_n273), .c(new_n269), .out0(\s[27] ));
  aoi112aa1n06x5               g179(.a(new_n268), .b(new_n234), .c(new_n232), .d(new_n184), .o1(new_n275));
  inv000aa1n02x5               g180(.a(new_n270), .o1(new_n276));
  aoai13aa1n02x7               g181(.a(new_n272), .b(new_n276), .c(new_n249), .d(new_n251), .o1(new_n277));
  oabi12aa1n03x5               g182(.a(new_n266), .b(new_n275), .c(new_n277), .out0(new_n278));
  inv000aa1n03x5               g183(.a(new_n265), .o1(new_n279));
  aoai13aa1n02x7               g184(.a(new_n279), .b(new_n266), .c(new_n273), .d(new_n269), .o1(new_n280));
  xorc02aa1n02x5               g185(.a(\a[28] ), .b(\b[27] ), .out0(new_n281));
  norp02aa1n02x5               g186(.a(new_n281), .b(new_n265), .o1(new_n282));
  aoi022aa1n03x5               g187(.a(new_n280), .b(new_n281), .c(new_n278), .d(new_n282), .o1(\s[28] ));
  inv000aa1d42x5               g188(.a(\a[27] ), .o1(new_n284));
  inv000aa1d42x5               g189(.a(\a[28] ), .o1(new_n285));
  xroi22aa1d06x4               g190(.a(new_n284), .b(\b[26] ), .c(new_n285), .d(\b[27] ), .out0(new_n286));
  oai012aa1n02x5               g191(.a(new_n286), .b(new_n275), .c(new_n277), .o1(new_n287));
  inv000aa1n06x5               g192(.a(new_n286), .o1(new_n288));
  oao003aa1n02x5               g193(.a(\a[28] ), .b(\b[27] ), .c(new_n279), .carry(new_n289));
  aoai13aa1n02x7               g194(.a(new_n289), .b(new_n288), .c(new_n273), .d(new_n269), .o1(new_n290));
  xorc02aa1n02x5               g195(.a(\a[29] ), .b(\b[28] ), .out0(new_n291));
  norb02aa1n02x5               g196(.a(new_n289), .b(new_n291), .out0(new_n292));
  aoi022aa1n03x5               g197(.a(new_n290), .b(new_n291), .c(new_n287), .d(new_n292), .o1(\s[29] ));
  nanp02aa1n02x5               g198(.a(\b[0] ), .b(\a[1] ), .o1(new_n294));
  xorb03aa1n02x5               g199(.a(new_n294), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  and003aa1n02x5               g200(.a(new_n281), .b(new_n291), .c(new_n267), .o(new_n296));
  oai012aa1n03x5               g201(.a(new_n296), .b(new_n275), .c(new_n277), .o1(new_n297));
  inv000aa1n02x5               g202(.a(new_n296), .o1(new_n298));
  oao003aa1n02x5               g203(.a(\a[29] ), .b(\b[28] ), .c(new_n289), .carry(new_n299));
  aoai13aa1n03x5               g204(.a(new_n299), .b(new_n298), .c(new_n273), .d(new_n269), .o1(new_n300));
  xorc02aa1n02x5               g205(.a(\a[30] ), .b(\b[29] ), .out0(new_n301));
  norb02aa1n02x5               g206(.a(new_n299), .b(new_n301), .out0(new_n302));
  aoi022aa1n03x5               g207(.a(new_n300), .b(new_n301), .c(new_n297), .d(new_n302), .o1(\s[30] ));
  nano22aa1n02x4               g208(.a(new_n288), .b(new_n291), .c(new_n301), .out0(new_n304));
  oai012aa1n02x5               g209(.a(new_n304), .b(new_n275), .c(new_n277), .o1(new_n305));
  xorc02aa1n02x5               g210(.a(\a[31] ), .b(\b[30] ), .out0(new_n306));
  and002aa1n02x5               g211(.a(\b[29] ), .b(\a[30] ), .o(new_n307));
  oabi12aa1n02x5               g212(.a(new_n306), .b(\a[30] ), .c(\b[29] ), .out0(new_n308));
  oab012aa1n02x4               g213(.a(new_n308), .b(new_n299), .c(new_n307), .out0(new_n309));
  inv000aa1n02x5               g214(.a(new_n304), .o1(new_n310));
  oao003aa1n02x5               g215(.a(\a[30] ), .b(\b[29] ), .c(new_n299), .carry(new_n311));
  aoai13aa1n02x7               g216(.a(new_n311), .b(new_n310), .c(new_n273), .d(new_n269), .o1(new_n312));
  aoi022aa1n03x5               g217(.a(new_n312), .b(new_n306), .c(new_n305), .d(new_n309), .o1(\s[31] ));
  inv000aa1n02x5               g218(.a(new_n102), .o1(new_n314));
  aob012aa1n03x5               g219(.a(new_n294), .b(\b[1] ), .c(\a[2] ), .out0(new_n315));
  xnbna2aa1n03x5               g220(.a(new_n101), .b(new_n315), .c(new_n314), .out0(\s[3] ));
  inv000aa1d42x5               g221(.a(\a[4] ), .o1(new_n317));
  inv000aa1d42x5               g222(.a(\b[3] ), .o1(new_n318));
  nanp02aa1n02x5               g223(.a(new_n318), .b(new_n317), .o1(new_n319));
  nanp02aa1n02x5               g224(.a(new_n315), .b(new_n314), .o1(new_n320));
  aoi122aa1n02x5               g225(.a(new_n99), .b(new_n106), .c(new_n319), .d(new_n320), .e(new_n100), .o1(new_n321));
  aoi013aa1n02x4               g226(.a(new_n321), .b(new_n128), .c(new_n106), .d(new_n319), .o1(\s[4] ));
  aoai13aa1n06x5               g227(.a(new_n106), .b(new_n104), .c(new_n320), .d(new_n101), .o1(new_n323));
  xnrb03aa1n02x5               g228(.a(new_n323), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  tech160nm_fioaoi03aa1n03p5x5 g229(.a(\a[5] ), .b(\b[4] ), .c(new_n323), .o1(new_n325));
  xorb03aa1n02x5               g230(.a(new_n325), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  nona22aa1n03x5               g231(.a(new_n107), .b(new_n325), .c(new_n115), .out0(new_n327));
  oai012aa1n02x5               g232(.a(new_n107), .b(new_n325), .c(new_n115), .o1(new_n328));
  nano22aa1n02x4               g233(.a(new_n109), .b(new_n107), .c(new_n110), .out0(new_n329));
  aoi022aa1n02x5               g234(.a(new_n327), .b(new_n329), .c(new_n111), .d(new_n328), .o1(\s[7] ));
  aoi012aa1n03x5               g235(.a(new_n109), .b(new_n327), .c(new_n329), .o1(new_n331));
  xnrb03aa1n03x5               g236(.a(new_n331), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xorb03aa1n02x5               g237(.a(new_n124), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


