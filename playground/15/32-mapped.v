// Benchmark "adder" written by ABC on Wed Jul 17 19:55:53 2024

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
    new_n132, new_n133, new_n134, new_n135, new_n136, new_n137, new_n138,
    new_n139, new_n140, new_n141, new_n143, new_n144, new_n145, new_n147,
    new_n148, new_n149, new_n150, new_n151, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n161, new_n163,
    new_n164, new_n165, new_n167, new_n168, new_n169, new_n170, new_n171,
    new_n172, new_n173, new_n174, new_n175, new_n176, new_n177, new_n178,
    new_n180, new_n181, new_n182, new_n183, new_n184, new_n185, new_n186,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n196, new_n198, new_n199, new_n200, new_n202, new_n203,
    new_n204, new_n205, new_n206, new_n207, new_n210, new_n211, new_n212,
    new_n213, new_n214, new_n215, new_n216, new_n218, new_n219, new_n220,
    new_n221, new_n222, new_n223, new_n225, new_n226, new_n227, new_n228,
    new_n229, new_n230, new_n232, new_n233, new_n234, new_n235, new_n236,
    new_n237, new_n238, new_n239, new_n241, new_n242, new_n243, new_n244,
    new_n245, new_n246, new_n247, new_n249, new_n250, new_n251, new_n252,
    new_n253, new_n254, new_n255, new_n256, new_n258, new_n259, new_n260,
    new_n261, new_n262, new_n264, new_n265, new_n266, new_n267, new_n268,
    new_n269, new_n270, new_n271, new_n272, new_n273, new_n274, new_n275,
    new_n276, new_n277, new_n279, new_n280, new_n281, new_n282, new_n283,
    new_n284, new_n285, new_n286, new_n287, new_n288, new_n289, new_n290,
    new_n291, new_n293, new_n294, new_n295, new_n296, new_n297, new_n298,
    new_n299, new_n302, new_n303, new_n304, new_n305, new_n306, new_n307,
    new_n308, new_n310, new_n311, new_n312, new_n313, new_n314, new_n315,
    new_n316, new_n319, new_n320, new_n322, new_n323, new_n325, new_n326,
    new_n328, new_n330;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor002aa1n06x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  xorc02aa1n02x5               g002(.a(\a[3] ), .b(\b[2] ), .out0(new_n98));
  orn002aa1n02x5               g003(.a(\a[2] ), .b(\b[1] ), .o(new_n99));
  nanp02aa1n02x5               g004(.a(\b[0] ), .b(\a[1] ), .o1(new_n100));
  aob012aa1n02x5               g005(.a(new_n100), .b(\b[1] ), .c(\a[2] ), .out0(new_n101));
  nanp02aa1n02x5               g006(.a(new_n101), .b(new_n99), .o1(new_n102));
  nor022aa1n08x5               g007(.a(\b[2] ), .b(\a[3] ), .o1(new_n103));
  nand42aa1n16x5               g008(.a(\b[3] ), .b(\a[4] ), .o1(new_n104));
  nor002aa1n06x5               g009(.a(\b[3] ), .b(\a[4] ), .o1(new_n105));
  norb03aa1d15x5               g010(.a(new_n104), .b(new_n103), .c(new_n105), .out0(new_n106));
  inv000aa1d42x5               g011(.a(new_n106), .o1(new_n107));
  aoi012aa1n02x5               g012(.a(new_n107), .b(new_n102), .c(new_n98), .o1(new_n108));
  nand42aa1n10x5               g013(.a(\b[5] ), .b(\a[6] ), .o1(new_n109));
  nor042aa1n06x5               g014(.a(\b[6] ), .b(\a[7] ), .o1(new_n110));
  nand42aa1n03x5               g015(.a(\b[6] ), .b(\a[7] ), .o1(new_n111));
  nano22aa1n03x7               g016(.a(new_n110), .b(new_n109), .c(new_n111), .out0(new_n112));
  nanp02aa1n02x5               g017(.a(\b[7] ), .b(\a[8] ), .o1(new_n113));
  inv000aa1d42x5               g018(.a(\a[8] ), .o1(new_n114));
  inv000aa1d42x5               g019(.a(\b[7] ), .o1(new_n115));
  nanp02aa1n02x5               g020(.a(new_n115), .b(new_n114), .o1(new_n116));
  nanp02aa1n02x5               g021(.a(new_n116), .b(new_n113), .o1(new_n117));
  nor002aa1n16x5               g022(.a(\b[5] ), .b(\a[6] ), .o1(new_n118));
  norp02aa1n06x5               g023(.a(\b[4] ), .b(\a[5] ), .o1(new_n119));
  nand42aa1n03x5               g024(.a(\b[4] ), .b(\a[5] ), .o1(new_n120));
  nano22aa1n02x4               g025(.a(new_n119), .b(new_n104), .c(new_n120), .out0(new_n121));
  nona23aa1n02x4               g026(.a(new_n121), .b(new_n112), .c(new_n117), .d(new_n118), .out0(new_n122));
  xorc02aa1n02x5               g027(.a(\a[8] ), .b(\b[7] ), .out0(new_n123));
  inv000aa1d42x5               g028(.a(new_n118), .o1(new_n124));
  oai112aa1n02x5               g029(.a(new_n124), .b(new_n109), .c(\b[4] ), .d(\a[5] ), .o1(new_n125));
  oao003aa1n12x5               g030(.a(new_n114), .b(new_n115), .c(new_n110), .carry(new_n126));
  aoi013aa1n03x5               g031(.a(new_n126), .b(new_n112), .c(new_n125), .d(new_n123), .o1(new_n127));
  oai012aa1n03x5               g032(.a(new_n127), .b(new_n122), .c(new_n108), .o1(new_n128));
  nand42aa1d28x5               g033(.a(\b[8] ), .b(\a[9] ), .o1(new_n129));
  aoi012aa1n02x5               g034(.a(new_n97), .b(new_n128), .c(new_n129), .o1(new_n130));
  norp02aa1n06x5               g035(.a(\b[9] ), .b(\a[10] ), .o1(new_n131));
  nand02aa1n06x5               g036(.a(\b[9] ), .b(\a[10] ), .o1(new_n132));
  norb02aa1n02x5               g037(.a(new_n132), .b(new_n131), .out0(new_n133));
  xnrc02aa1n02x5               g038(.a(\b[2] ), .b(\a[3] ), .out0(new_n134));
  aoai13aa1n06x5               g039(.a(new_n106), .b(new_n134), .c(new_n101), .d(new_n99), .o1(new_n135));
  nanb03aa1n09x5               g040(.a(new_n110), .b(new_n111), .c(new_n109), .out0(new_n136));
  nona23aa1n02x4               g041(.a(new_n104), .b(new_n120), .c(new_n119), .d(new_n118), .out0(new_n137));
  nona32aa1n02x4               g042(.a(new_n135), .b(new_n137), .c(new_n117), .d(new_n136), .out0(new_n138));
  nanb02aa1n02x5               g043(.a(new_n97), .b(new_n129), .out0(new_n139));
  norb03aa1n06x5               g044(.a(new_n132), .b(new_n97), .c(new_n131), .out0(new_n140));
  aoai13aa1n02x5               g045(.a(new_n140), .b(new_n139), .c(new_n138), .d(new_n127), .o1(new_n141));
  oai012aa1n02x5               g046(.a(new_n141), .b(new_n130), .c(new_n133), .o1(\s[10] ));
  nand42aa1n04x5               g047(.a(\b[10] ), .b(\a[11] ), .o1(new_n143));
  nor022aa1n16x5               g048(.a(\b[10] ), .b(\a[11] ), .o1(new_n144));
  norb02aa1n02x5               g049(.a(new_n143), .b(new_n144), .out0(new_n145));
  xobna2aa1n03x5               g050(.a(new_n145), .b(new_n141), .c(new_n132), .out0(\s[11] ));
  inv000aa1d42x5               g051(.a(new_n144), .o1(new_n147));
  nanp03aa1n02x5               g052(.a(new_n141), .b(new_n132), .c(new_n145), .o1(new_n148));
  nor042aa1n04x5               g053(.a(\b[11] ), .b(\a[12] ), .o1(new_n149));
  nand42aa1n06x5               g054(.a(\b[11] ), .b(\a[12] ), .o1(new_n150));
  norb02aa1n12x5               g055(.a(new_n150), .b(new_n149), .out0(new_n151));
  xnbna2aa1n03x5               g056(.a(new_n151), .b(new_n148), .c(new_n147), .out0(\s[12] ));
  nano22aa1n03x7               g057(.a(new_n144), .b(new_n132), .c(new_n143), .out0(new_n153));
  norb03aa1n06x5               g058(.a(new_n129), .b(new_n97), .c(new_n131), .out0(new_n154));
  nand23aa1n09x5               g059(.a(new_n153), .b(new_n154), .c(new_n151), .o1(new_n155));
  nanb02aa1n03x5               g060(.a(new_n149), .b(new_n150), .out0(new_n156));
  nanb03aa1n06x5               g061(.a(new_n144), .b(new_n132), .c(new_n143), .out0(new_n157));
  oaih12aa1n02x5               g062(.a(new_n150), .b(new_n149), .c(new_n144), .o1(new_n158));
  oai013aa1d12x5               g063(.a(new_n158), .b(new_n140), .c(new_n157), .d(new_n156), .o1(new_n159));
  inv000aa1d42x5               g064(.a(new_n159), .o1(new_n160));
  aoai13aa1n02x5               g065(.a(new_n160), .b(new_n155), .c(new_n138), .d(new_n127), .o1(new_n161));
  xorb03aa1n02x5               g066(.a(new_n161), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  nor002aa1n04x5               g067(.a(\b[12] ), .b(\a[13] ), .o1(new_n163));
  nand42aa1d28x5               g068(.a(\b[12] ), .b(\a[13] ), .o1(new_n164));
  aoi012aa1n02x5               g069(.a(new_n163), .b(new_n161), .c(new_n164), .o1(new_n165));
  xnrb03aa1n02x5               g070(.a(new_n165), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  nor043aa1n02x5               g071(.a(new_n137), .b(new_n117), .c(new_n136), .o1(new_n167));
  norb03aa1n03x5               g072(.a(new_n109), .b(new_n119), .c(new_n118), .out0(new_n168));
  inv000aa1n02x5               g073(.a(new_n126), .o1(new_n169));
  oai013aa1n03x5               g074(.a(new_n169), .b(new_n168), .c(new_n136), .d(new_n117), .o1(new_n170));
  inv000aa1d42x5               g075(.a(new_n155), .o1(new_n171));
  aoai13aa1n03x5               g076(.a(new_n171), .b(new_n170), .c(new_n167), .d(new_n135), .o1(new_n172));
  nor002aa1n03x5               g077(.a(\b[13] ), .b(\a[14] ), .o1(new_n173));
  nand42aa1n08x5               g078(.a(\b[13] ), .b(\a[14] ), .o1(new_n174));
  nano23aa1d12x5               g079(.a(new_n163), .b(new_n173), .c(new_n174), .d(new_n164), .out0(new_n175));
  inv000aa1d42x5               g080(.a(new_n175), .o1(new_n176));
  oaih12aa1n02x5               g081(.a(new_n174), .b(new_n173), .c(new_n163), .o1(new_n177));
  aoai13aa1n04x5               g082(.a(new_n177), .b(new_n176), .c(new_n172), .d(new_n160), .o1(new_n178));
  xorb03aa1n02x5               g083(.a(new_n178), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  norp02aa1n04x5               g084(.a(\b[14] ), .b(\a[15] ), .o1(new_n180));
  nand42aa1n03x5               g085(.a(\b[14] ), .b(\a[15] ), .o1(new_n181));
  norp02aa1n04x5               g086(.a(\b[15] ), .b(\a[16] ), .o1(new_n182));
  nand42aa1n02x5               g087(.a(\b[15] ), .b(\a[16] ), .o1(new_n183));
  nanb02aa1n02x5               g088(.a(new_n182), .b(new_n183), .out0(new_n184));
  aoai13aa1n02x5               g089(.a(new_n184), .b(new_n180), .c(new_n178), .d(new_n181), .o1(new_n185));
  aoi112aa1n03x5               g090(.a(new_n180), .b(new_n184), .c(new_n178), .d(new_n181), .o1(new_n186));
  nanb02aa1n03x5               g091(.a(new_n186), .b(new_n185), .out0(\s[16] ));
  nano23aa1n03x7               g092(.a(new_n180), .b(new_n182), .c(new_n183), .d(new_n181), .out0(new_n188));
  nano22aa1n03x7               g093(.a(new_n155), .b(new_n175), .c(new_n188), .out0(new_n189));
  aoai13aa1n12x5               g094(.a(new_n189), .b(new_n170), .c(new_n167), .d(new_n135), .o1(new_n190));
  nona23aa1n02x5               g095(.a(new_n183), .b(new_n181), .c(new_n180), .d(new_n182), .out0(new_n191));
  norb02aa1n03x5               g096(.a(new_n175), .b(new_n191), .out0(new_n192));
  tech160nm_fiao0012aa1n02p5x5 g097(.a(new_n182), .b(new_n180), .c(new_n183), .o(new_n193));
  oabi12aa1n06x5               g098(.a(new_n193), .b(new_n191), .c(new_n177), .out0(new_n194));
  aoi012aa1d18x5               g099(.a(new_n194), .b(new_n159), .c(new_n192), .o1(new_n195));
  xnrc02aa1n02x5               g100(.a(\b[16] ), .b(\a[17] ), .out0(new_n196));
  xobna2aa1n03x5               g101(.a(new_n196), .b(new_n190), .c(new_n195), .out0(\s[17] ));
  inv000aa1d42x5               g102(.a(\a[17] ), .o1(new_n198));
  nanb02aa1n02x5               g103(.a(\b[16] ), .b(new_n198), .out0(new_n199));
  aoai13aa1n02x5               g104(.a(new_n199), .b(new_n196), .c(new_n190), .d(new_n195), .o1(new_n200));
  xorb03aa1n02x5               g105(.a(new_n200), .b(\b[17] ), .c(\a[18] ), .out0(\s[18] ));
  inv040aa1d32x5               g106(.a(\a[18] ), .o1(new_n202));
  xroi22aa1d06x4               g107(.a(new_n198), .b(\b[16] ), .c(new_n202), .d(\b[17] ), .out0(new_n203));
  inv000aa1d42x5               g108(.a(new_n203), .o1(new_n204));
  oai022aa1n02x5               g109(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n205));
  oaib12aa1n02x5               g110(.a(new_n205), .b(new_n202), .c(\b[17] ), .out0(new_n206));
  aoai13aa1n04x5               g111(.a(new_n206), .b(new_n204), .c(new_n190), .d(new_n195), .o1(new_n207));
  xorb03aa1n02x5               g112(.a(new_n207), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g113(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor042aa1n02x5               g114(.a(\b[18] ), .b(\a[19] ), .o1(new_n210));
  nand42aa1n04x5               g115(.a(\b[18] ), .b(\a[19] ), .o1(new_n211));
  nor042aa1n02x5               g116(.a(\b[19] ), .b(\a[20] ), .o1(new_n212));
  nand42aa1n03x5               g117(.a(\b[19] ), .b(\a[20] ), .o1(new_n213));
  nanb02aa1n02x5               g118(.a(new_n212), .b(new_n213), .out0(new_n214));
  aoai13aa1n02x7               g119(.a(new_n214), .b(new_n210), .c(new_n207), .d(new_n211), .o1(new_n215));
  aoi112aa1n03x5               g120(.a(new_n210), .b(new_n214), .c(new_n207), .d(new_n211), .o1(new_n216));
  nanb02aa1n03x5               g121(.a(new_n216), .b(new_n215), .out0(\s[20] ));
  nano23aa1n09x5               g122(.a(new_n210), .b(new_n212), .c(new_n213), .d(new_n211), .out0(new_n218));
  nanp02aa1n02x5               g123(.a(new_n203), .b(new_n218), .o1(new_n219));
  oaoi03aa1n02x5               g124(.a(\a[18] ), .b(\b[17] ), .c(new_n199), .o1(new_n220));
  oai012aa1n02x5               g125(.a(new_n213), .b(new_n212), .c(new_n210), .o1(new_n221));
  aobi12aa1n03x5               g126(.a(new_n221), .b(new_n218), .c(new_n220), .out0(new_n222));
  aoai13aa1n06x5               g127(.a(new_n222), .b(new_n219), .c(new_n190), .d(new_n195), .o1(new_n223));
  xorb03aa1n02x5               g128(.a(new_n223), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  norp02aa1n02x5               g129(.a(\b[20] ), .b(\a[21] ), .o1(new_n225));
  xnrc02aa1n12x5               g130(.a(\b[20] ), .b(\a[21] ), .out0(new_n226));
  inv000aa1d42x5               g131(.a(new_n226), .o1(new_n227));
  tech160nm_fixnrc02aa1n04x5   g132(.a(\b[21] ), .b(\a[22] ), .out0(new_n228));
  aoai13aa1n03x5               g133(.a(new_n228), .b(new_n225), .c(new_n223), .d(new_n227), .o1(new_n229));
  aoi112aa1n03x4               g134(.a(new_n225), .b(new_n228), .c(new_n223), .d(new_n227), .o1(new_n230));
  nanb02aa1n03x5               g135(.a(new_n230), .b(new_n229), .out0(\s[22] ));
  nor042aa1n02x5               g136(.a(new_n228), .b(new_n226), .o1(new_n232));
  nand23aa1n06x5               g137(.a(new_n203), .b(new_n232), .c(new_n218), .o1(new_n233));
  nona23aa1n02x4               g138(.a(new_n213), .b(new_n211), .c(new_n210), .d(new_n212), .out0(new_n234));
  oai012aa1n02x5               g139(.a(new_n221), .b(new_n234), .c(new_n206), .o1(new_n235));
  inv000aa1n02x5               g140(.a(new_n225), .o1(new_n236));
  oaoi03aa1n02x5               g141(.a(\a[22] ), .b(\b[21] ), .c(new_n236), .o1(new_n237));
  aoi012aa1n02x5               g142(.a(new_n237), .b(new_n235), .c(new_n232), .o1(new_n238));
  aoai13aa1n04x5               g143(.a(new_n238), .b(new_n233), .c(new_n190), .d(new_n195), .o1(new_n239));
  xorb03aa1n02x5               g144(.a(new_n239), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n04x5               g145(.a(\b[22] ), .b(\a[23] ), .o1(new_n241));
  nanp02aa1n02x5               g146(.a(\b[22] ), .b(\a[23] ), .o1(new_n242));
  nor042aa1n02x5               g147(.a(\b[23] ), .b(\a[24] ), .o1(new_n243));
  nanp02aa1n02x5               g148(.a(\b[23] ), .b(\a[24] ), .o1(new_n244));
  nanb02aa1n02x5               g149(.a(new_n243), .b(new_n244), .out0(new_n245));
  aoai13aa1n02x5               g150(.a(new_n245), .b(new_n241), .c(new_n239), .d(new_n242), .o1(new_n246));
  aoi112aa1n02x7               g151(.a(new_n241), .b(new_n245), .c(new_n239), .d(new_n242), .o1(new_n247));
  nanb02aa1n03x5               g152(.a(new_n247), .b(new_n246), .out0(\s[24] ));
  nona23aa1n02x4               g153(.a(new_n244), .b(new_n242), .c(new_n241), .d(new_n243), .out0(new_n249));
  nona23aa1n02x4               g154(.a(new_n203), .b(new_n232), .c(new_n249), .d(new_n234), .out0(new_n250));
  nano23aa1n06x5               g155(.a(new_n241), .b(new_n243), .c(new_n244), .d(new_n242), .out0(new_n251));
  tech160nm_fiao0012aa1n02p5x5 g156(.a(new_n243), .b(new_n241), .c(new_n244), .o(new_n252));
  aoi012aa1n06x5               g157(.a(new_n252), .b(new_n251), .c(new_n237), .o1(new_n253));
  inv020aa1n03x5               g158(.a(new_n253), .o1(new_n254));
  aoi013aa1n02x4               g159(.a(new_n254), .b(new_n235), .c(new_n232), .d(new_n251), .o1(new_n255));
  aoai13aa1n04x5               g160(.a(new_n255), .b(new_n250), .c(new_n190), .d(new_n195), .o1(new_n256));
  xorb03aa1n02x5               g161(.a(new_n256), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g162(.a(\b[24] ), .b(\a[25] ), .o1(new_n258));
  xorc02aa1n03x5               g163(.a(\a[25] ), .b(\b[24] ), .out0(new_n259));
  tech160nm_fixnrc02aa1n04x5   g164(.a(\b[25] ), .b(\a[26] ), .out0(new_n260));
  aoai13aa1n03x5               g165(.a(new_n260), .b(new_n258), .c(new_n256), .d(new_n259), .o1(new_n261));
  aoi112aa1n03x4               g166(.a(new_n258), .b(new_n260), .c(new_n256), .d(new_n259), .o1(new_n262));
  nanb02aa1n03x5               g167(.a(new_n262), .b(new_n261), .out0(\s[26] ));
  nanb03aa1n02x5               g168(.a(new_n140), .b(new_n153), .c(new_n151), .out0(new_n264));
  nanp02aa1n02x5               g169(.a(new_n188), .b(new_n175), .o1(new_n265));
  aoib12aa1n02x5               g170(.a(new_n193), .b(new_n188), .c(new_n177), .out0(new_n266));
  aoai13aa1n02x5               g171(.a(new_n266), .b(new_n265), .c(new_n264), .d(new_n158), .o1(new_n267));
  norb02aa1n06x5               g172(.a(new_n259), .b(new_n260), .out0(new_n268));
  nano22aa1n03x7               g173(.a(new_n233), .b(new_n251), .c(new_n268), .out0(new_n269));
  aoai13aa1n06x5               g174(.a(new_n269), .b(new_n267), .c(new_n128), .d(new_n189), .o1(new_n270));
  nano22aa1n03x5               g175(.a(new_n222), .b(new_n232), .c(new_n251), .out0(new_n271));
  inv000aa1d42x5               g176(.a(\a[26] ), .o1(new_n272));
  inv000aa1d42x5               g177(.a(\b[25] ), .o1(new_n273));
  oaoi03aa1n02x5               g178(.a(new_n272), .b(new_n273), .c(new_n258), .o1(new_n274));
  inv000aa1n02x5               g179(.a(new_n274), .o1(new_n275));
  oaoi13aa1n09x5               g180(.a(new_n275), .b(new_n268), .c(new_n271), .d(new_n254), .o1(new_n276));
  xorc02aa1n02x5               g181(.a(\a[27] ), .b(\b[26] ), .out0(new_n277));
  xnbna2aa1n03x5               g182(.a(new_n277), .b(new_n276), .c(new_n270), .out0(\s[27] ));
  nand42aa1n03x5               g183(.a(new_n276), .b(new_n270), .o1(new_n279));
  norp02aa1n02x5               g184(.a(\b[26] ), .b(\a[27] ), .o1(new_n280));
  norp02aa1n02x5               g185(.a(\b[27] ), .b(\a[28] ), .o1(new_n281));
  nand42aa1n08x5               g186(.a(\b[27] ), .b(\a[28] ), .o1(new_n282));
  norb02aa1n02x7               g187(.a(new_n282), .b(new_n281), .out0(new_n283));
  inv000aa1d42x5               g188(.a(new_n283), .o1(new_n284));
  aoai13aa1n03x5               g189(.a(new_n284), .b(new_n280), .c(new_n279), .d(new_n277), .o1(new_n285));
  nand02aa1d06x5               g190(.a(new_n190), .b(new_n195), .o1(new_n286));
  nona32aa1n02x4               g191(.a(new_n235), .b(new_n249), .c(new_n228), .d(new_n226), .out0(new_n287));
  inv000aa1d42x5               g192(.a(new_n268), .o1(new_n288));
  aoai13aa1n06x5               g193(.a(new_n274), .b(new_n288), .c(new_n287), .d(new_n253), .o1(new_n289));
  aoai13aa1n03x5               g194(.a(new_n277), .b(new_n289), .c(new_n286), .d(new_n269), .o1(new_n290));
  nona22aa1n02x5               g195(.a(new_n290), .b(new_n284), .c(new_n280), .out0(new_n291));
  nanp02aa1n03x5               g196(.a(new_n285), .b(new_n291), .o1(\s[28] ));
  norb02aa1n02x5               g197(.a(new_n277), .b(new_n284), .out0(new_n293));
  aoai13aa1n03x5               g198(.a(new_n293), .b(new_n289), .c(new_n286), .d(new_n269), .o1(new_n294));
  oai012aa1n02x5               g199(.a(new_n282), .b(new_n281), .c(new_n280), .o1(new_n295));
  xnrc02aa1n02x5               g200(.a(\b[28] ), .b(\a[29] ), .out0(new_n296));
  tech160nm_fiaoi012aa1n02p5x5 g201(.a(new_n296), .b(new_n294), .c(new_n295), .o1(new_n297));
  aobi12aa1n02x7               g202(.a(new_n293), .b(new_n276), .c(new_n270), .out0(new_n298));
  nano22aa1n03x5               g203(.a(new_n298), .b(new_n295), .c(new_n296), .out0(new_n299));
  norp02aa1n03x5               g204(.a(new_n297), .b(new_n299), .o1(\s[29] ));
  xorb03aa1n02x5               g205(.a(new_n100), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n02x4               g206(.a(new_n296), .b(new_n277), .c(new_n283), .out0(new_n302));
  aoai13aa1n03x5               g207(.a(new_n302), .b(new_n289), .c(new_n286), .d(new_n269), .o1(new_n303));
  oao003aa1n02x5               g208(.a(\a[29] ), .b(\b[28] ), .c(new_n295), .carry(new_n304));
  xnrc02aa1n02x5               g209(.a(\b[29] ), .b(\a[30] ), .out0(new_n305));
  tech160nm_fiaoi012aa1n02p5x5 g210(.a(new_n305), .b(new_n303), .c(new_n304), .o1(new_n306));
  aobi12aa1n02x7               g211(.a(new_n302), .b(new_n276), .c(new_n270), .out0(new_n307));
  nano22aa1n03x5               g212(.a(new_n307), .b(new_n304), .c(new_n305), .out0(new_n308));
  norp02aa1n03x5               g213(.a(new_n306), .b(new_n308), .o1(\s[30] ));
  nano23aa1n06x5               g214(.a(new_n305), .b(new_n296), .c(new_n277), .d(new_n283), .out0(new_n310));
  aoai13aa1n03x5               g215(.a(new_n310), .b(new_n289), .c(new_n286), .d(new_n269), .o1(new_n311));
  oao003aa1n02x5               g216(.a(\a[30] ), .b(\b[29] ), .c(new_n304), .carry(new_n312));
  xnrc02aa1n02x5               g217(.a(\b[30] ), .b(\a[31] ), .out0(new_n313));
  tech160nm_fiaoi012aa1n02p5x5 g218(.a(new_n313), .b(new_n311), .c(new_n312), .o1(new_n314));
  aobi12aa1n02x7               g219(.a(new_n310), .b(new_n276), .c(new_n270), .out0(new_n315));
  nano22aa1n03x5               g220(.a(new_n315), .b(new_n312), .c(new_n313), .out0(new_n316));
  norp02aa1n03x5               g221(.a(new_n314), .b(new_n316), .o1(\s[31] ));
  xnbna2aa1n03x5               g222(.a(new_n98), .b(new_n101), .c(new_n99), .out0(\s[3] ));
  norb02aa1n02x5               g223(.a(new_n104), .b(new_n105), .out0(new_n319));
  aoi012aa1n02x5               g224(.a(new_n103), .b(new_n102), .c(new_n98), .o1(new_n320));
  oai012aa1n02x5               g225(.a(new_n135), .b(new_n320), .c(new_n319), .o1(\s[4] ));
  aoai13aa1n02x5               g226(.a(new_n121), .b(new_n107), .c(new_n102), .d(new_n98), .o1(new_n322));
  aboi22aa1n03x5               g227(.a(new_n119), .b(new_n120), .c(new_n135), .d(new_n104), .out0(new_n323));
  norb02aa1n02x5               g228(.a(new_n322), .b(new_n323), .out0(\s[5] ));
  aoi012aa1n02x5               g229(.a(new_n119), .b(new_n135), .c(new_n121), .o1(new_n325));
  nanp02aa1n02x5               g230(.a(new_n322), .b(new_n168), .o1(new_n326));
  aoai13aa1n02x5               g231(.a(new_n326), .b(new_n325), .c(new_n109), .d(new_n124), .o1(\s[6] ));
  norb02aa1n02x5               g232(.a(new_n111), .b(new_n110), .out0(new_n328));
  xobna2aa1n03x5               g233(.a(new_n328), .b(new_n326), .c(new_n109), .out0(\s[7] ));
  aoi012aa1n02x5               g234(.a(new_n110), .b(new_n326), .c(new_n112), .o1(new_n330));
  xnbna2aa1n03x5               g235(.a(new_n330), .b(new_n113), .c(new_n116), .out0(\s[8] ));
  xobna2aa1n03x5               g236(.a(new_n139), .b(new_n138), .c(new_n127), .out0(\s[9] ));
endmodule


