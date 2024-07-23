// Benchmark "adder" written by ABC on Wed Jul 17 21:18:19 2024

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
    new_n126, new_n127, new_n128, new_n129, new_n130, new_n131, new_n132,
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n138, new_n139,
    new_n140, new_n141, new_n143, new_n144, new_n145, new_n146, new_n147,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n158, new_n159, new_n160, new_n161, new_n163, new_n164,
    new_n165, new_n166, new_n167, new_n168, new_n169, new_n171, new_n172,
    new_n173, new_n174, new_n175, new_n176, new_n177, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n189, new_n190, new_n191, new_n192, new_n194, new_n195, new_n196,
    new_n197, new_n198, new_n199, new_n200, new_n203, new_n204, new_n205,
    new_n206, new_n207, new_n208, new_n209, new_n211, new_n212, new_n213,
    new_n214, new_n215, new_n216, new_n217, new_n218, new_n219, new_n220,
    new_n222, new_n223, new_n224, new_n225, new_n226, new_n227, new_n228,
    new_n229, new_n231, new_n232, new_n233, new_n234, new_n235, new_n237,
    new_n238, new_n239, new_n240, new_n241, new_n242, new_n243, new_n244,
    new_n245, new_n246, new_n247, new_n248, new_n250, new_n251, new_n252,
    new_n253, new_n254, new_n255, new_n256, new_n257, new_n259, new_n260,
    new_n261, new_n262, new_n263, new_n265, new_n266, new_n267, new_n268,
    new_n269, new_n270, new_n271, new_n272, new_n273, new_n274, new_n275,
    new_n276, new_n277, new_n278, new_n279, new_n280, new_n281, new_n282,
    new_n284, new_n285, new_n286, new_n287, new_n288, new_n289, new_n290,
    new_n291, new_n292, new_n293, new_n294, new_n296, new_n297, new_n298,
    new_n299, new_n300, new_n301, new_n302, new_n303, new_n306, new_n307,
    new_n308, new_n309, new_n310, new_n311, new_n312, new_n313, new_n315,
    new_n316, new_n317, new_n318, new_n319, new_n320, new_n321, new_n322,
    new_n325, new_n328, new_n330, new_n331, new_n333;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  norp02aa1n02x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  nor042aa1n02x5               g002(.a(\b[7] ), .b(\a[8] ), .o1(new_n98));
  nanp02aa1n04x5               g003(.a(\b[7] ), .b(\a[8] ), .o1(new_n99));
  norb02aa1n06x4               g004(.a(new_n99), .b(new_n98), .out0(new_n100));
  xorc02aa1n12x5               g005(.a(\a[7] ), .b(\b[6] ), .out0(new_n101));
  nor002aa1n04x5               g006(.a(\b[5] ), .b(\a[6] ), .o1(new_n102));
  nanp02aa1n02x5               g007(.a(\b[5] ), .b(\a[6] ), .o1(new_n103));
  norp02aa1n02x5               g008(.a(\b[4] ), .b(\a[5] ), .o1(new_n104));
  nanp02aa1n02x5               g009(.a(\b[4] ), .b(\a[5] ), .o1(new_n105));
  nona23aa1n02x4               g010(.a(new_n105), .b(new_n103), .c(new_n102), .d(new_n104), .out0(new_n106));
  nanb03aa1n02x5               g011(.a(new_n106), .b(new_n100), .c(new_n101), .out0(new_n107));
  nor002aa1n02x5               g012(.a(\b[3] ), .b(\a[4] ), .o1(new_n108));
  nanp02aa1n02x5               g013(.a(\b[3] ), .b(\a[4] ), .o1(new_n109));
  norp02aa1n04x5               g014(.a(\b[2] ), .b(\a[3] ), .o1(new_n110));
  nanp02aa1n02x5               g015(.a(\b[2] ), .b(\a[3] ), .o1(new_n111));
  nano23aa1n02x4               g016(.a(new_n108), .b(new_n110), .c(new_n111), .d(new_n109), .out0(new_n112));
  inv000aa1d42x5               g017(.a(\a[2] ), .o1(new_n113));
  inv000aa1d42x5               g018(.a(\b[1] ), .o1(new_n114));
  nand42aa1n02x5               g019(.a(\b[0] ), .b(\a[1] ), .o1(new_n115));
  oao003aa1n02x5               g020(.a(new_n113), .b(new_n114), .c(new_n115), .carry(new_n116));
  oai012aa1n02x5               g021(.a(new_n109), .b(new_n110), .c(new_n108), .o1(new_n117));
  aobi12aa1n02x5               g022(.a(new_n117), .b(new_n112), .c(new_n116), .out0(new_n118));
  aoi112aa1n02x5               g023(.a(\b[6] ), .b(\a[7] ), .c(\a[8] ), .d(\b[7] ), .o1(new_n119));
  tech160nm_fiao0012aa1n02p5x5 g024(.a(new_n102), .b(new_n104), .c(new_n103), .o(new_n120));
  aoi113aa1n02x5               g025(.a(new_n119), .b(new_n98), .c(new_n120), .d(new_n101), .e(new_n99), .o1(new_n121));
  tech160nm_fioai012aa1n05x5   g026(.a(new_n121), .b(new_n118), .c(new_n107), .o1(new_n122));
  nand42aa1n02x5               g027(.a(\b[8] ), .b(\a[9] ), .o1(new_n123));
  aoi012aa1n02x5               g028(.a(new_n97), .b(new_n122), .c(new_n123), .o1(new_n124));
  xnrb03aa1n02x5               g029(.a(new_n124), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  nano22aa1n03x7               g030(.a(new_n106), .b(new_n101), .c(new_n100), .out0(new_n126));
  nona23aa1n03x5               g031(.a(new_n111), .b(new_n109), .c(new_n108), .d(new_n110), .out0(new_n127));
  tech160nm_fioaoi03aa1n02p5x5 g032(.a(new_n113), .b(new_n114), .c(new_n115), .o1(new_n128));
  tech160nm_fioai012aa1n04x5   g033(.a(new_n117), .b(new_n127), .c(new_n128), .o1(new_n129));
  aoi112aa1n02x5               g034(.a(\b[4] ), .b(\a[5] ), .c(\a[6] ), .d(\b[5] ), .o1(new_n130));
  oai112aa1n02x5               g035(.a(new_n101), .b(new_n100), .c(new_n102), .d(new_n130), .o1(new_n131));
  nona22aa1n02x4               g036(.a(new_n131), .b(new_n119), .c(new_n98), .out0(new_n132));
  nor002aa1n02x5               g037(.a(\b[9] ), .b(\a[10] ), .o1(new_n133));
  nand42aa1n03x5               g038(.a(\b[9] ), .b(\a[10] ), .o1(new_n134));
  nano23aa1n02x5               g039(.a(new_n133), .b(new_n97), .c(new_n123), .d(new_n134), .out0(new_n135));
  aoai13aa1n06x5               g040(.a(new_n135), .b(new_n132), .c(new_n129), .d(new_n126), .o1(new_n136));
  tech160nm_fiao0012aa1n02p5x5 g041(.a(new_n133), .b(new_n97), .c(new_n134), .o(new_n137));
  inv000aa1d42x5               g042(.a(new_n137), .o1(new_n138));
  nor042aa1d18x5               g043(.a(\b[10] ), .b(\a[11] ), .o1(new_n139));
  nanp02aa1n09x5               g044(.a(\b[10] ), .b(\a[11] ), .o1(new_n140));
  norb02aa1n06x5               g045(.a(new_n140), .b(new_n139), .out0(new_n141));
  xnbna2aa1n03x5               g046(.a(new_n141), .b(new_n136), .c(new_n138), .out0(\s[11] ));
  inv000aa1d42x5               g047(.a(new_n139), .o1(new_n143));
  aoai13aa1n02x5               g048(.a(new_n141), .b(new_n137), .c(new_n122), .d(new_n135), .o1(new_n144));
  nor042aa1n04x5               g049(.a(\b[11] ), .b(\a[12] ), .o1(new_n145));
  nand02aa1n06x5               g050(.a(\b[11] ), .b(\a[12] ), .o1(new_n146));
  norb02aa1n06x4               g051(.a(new_n146), .b(new_n145), .out0(new_n147));
  xnbna2aa1n03x5               g052(.a(new_n147), .b(new_n144), .c(new_n143), .out0(\s[12] ));
  nano23aa1n09x5               g053(.a(new_n139), .b(new_n145), .c(new_n146), .d(new_n140), .out0(new_n149));
  inv000aa1d42x5               g054(.a(new_n149), .o1(new_n150));
  aoi112aa1n02x5               g055(.a(\b[10] ), .b(\a[11] ), .c(\a[12] ), .d(\b[11] ), .o1(new_n151));
  aoi112aa1n09x5               g056(.a(\b[8] ), .b(\a[9] ), .c(\a[10] ), .d(\b[9] ), .o1(new_n152));
  oai112aa1n04x5               g057(.a(new_n141), .b(new_n147), .c(new_n152), .d(new_n133), .o1(new_n153));
  nona22aa1d18x5               g058(.a(new_n153), .b(new_n151), .c(new_n145), .out0(new_n154));
  inv000aa1d42x5               g059(.a(new_n154), .o1(new_n155));
  oai012aa1n02x5               g060(.a(new_n155), .b(new_n136), .c(new_n150), .o1(new_n156));
  xorb03aa1n02x5               g061(.a(new_n156), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  inv000aa1d42x5               g062(.a(\a[14] ), .o1(new_n158));
  nor042aa1n06x5               g063(.a(\b[12] ), .b(\a[13] ), .o1(new_n159));
  nand02aa1n08x5               g064(.a(\b[12] ), .b(\a[13] ), .o1(new_n160));
  aoi012aa1n03x5               g065(.a(new_n159), .b(new_n156), .c(new_n160), .o1(new_n161));
  xorb03aa1n02x5               g066(.a(new_n161), .b(\b[13] ), .c(new_n158), .out0(\s[14] ));
  nor042aa1n04x5               g067(.a(\b[13] ), .b(\a[14] ), .o1(new_n163));
  nanp02aa1n06x5               g068(.a(\b[13] ), .b(\a[14] ), .o1(new_n164));
  nano23aa1d15x5               g069(.a(new_n159), .b(new_n163), .c(new_n164), .d(new_n160), .out0(new_n165));
  inv000aa1d42x5               g070(.a(new_n165), .o1(new_n166));
  aoi012aa1n02x7               g071(.a(new_n163), .b(new_n159), .c(new_n164), .o1(new_n167));
  nand22aa1n03x5               g072(.a(new_n165), .b(new_n149), .o1(new_n168));
  oai122aa1n02x7               g073(.a(new_n167), .b(new_n136), .c(new_n168), .d(new_n155), .e(new_n166), .o1(new_n169));
  xorb03aa1n02x5               g074(.a(new_n169), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  nor042aa1n04x5               g075(.a(\b[14] ), .b(\a[15] ), .o1(new_n171));
  nand42aa1d28x5               g076(.a(\b[14] ), .b(\a[15] ), .o1(new_n172));
  nor042aa1n06x5               g077(.a(\b[15] ), .b(\a[16] ), .o1(new_n173));
  nand42aa1n10x5               g078(.a(\b[15] ), .b(\a[16] ), .o1(new_n174));
  norb02aa1n02x5               g079(.a(new_n174), .b(new_n173), .out0(new_n175));
  aoi112aa1n02x5               g080(.a(new_n171), .b(new_n175), .c(new_n169), .d(new_n172), .o1(new_n176));
  aoai13aa1n02x5               g081(.a(new_n175), .b(new_n171), .c(new_n169), .d(new_n172), .o1(new_n177));
  norb02aa1n02x7               g082(.a(new_n177), .b(new_n176), .out0(\s[16] ));
  nano23aa1d15x5               g083(.a(new_n171), .b(new_n173), .c(new_n174), .d(new_n172), .out0(new_n179));
  aoi112aa1n02x5               g084(.a(\b[14] ), .b(\a[15] ), .c(\a[16] ), .d(\b[15] ), .o1(new_n180));
  inv020aa1n03x5               g085(.a(new_n167), .o1(new_n181));
  nand42aa1n06x5               g086(.a(new_n179), .b(new_n181), .o1(new_n182));
  nona22aa1d18x5               g087(.a(new_n182), .b(new_n180), .c(new_n173), .out0(new_n183));
  aoi013aa1n09x5               g088(.a(new_n183), .b(new_n154), .c(new_n165), .d(new_n179), .o1(new_n184));
  nano22aa1n03x7               g089(.a(new_n168), .b(new_n135), .c(new_n179), .out0(new_n185));
  aoai13aa1n06x5               g090(.a(new_n185), .b(new_n132), .c(new_n129), .d(new_n126), .o1(new_n186));
  nanp02aa1n09x5               g091(.a(new_n184), .b(new_n186), .o1(new_n187));
  xorb03aa1n02x5               g092(.a(new_n187), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv040aa1d32x5               g093(.a(\a[18] ), .o1(new_n189));
  inv000aa1d42x5               g094(.a(\a[17] ), .o1(new_n190));
  inv000aa1d42x5               g095(.a(\b[16] ), .o1(new_n191));
  oaoi03aa1n03x5               g096(.a(new_n190), .b(new_n191), .c(new_n187), .o1(new_n192));
  xorb03aa1n02x5               g097(.a(new_n192), .b(\b[17] ), .c(new_n189), .out0(\s[18] ));
  xroi22aa1d06x4               g098(.a(new_n190), .b(\b[16] ), .c(new_n189), .d(\b[17] ), .out0(new_n194));
  inv000aa1d42x5               g099(.a(new_n194), .o1(new_n195));
  inv000aa1d42x5               g100(.a(\b[17] ), .o1(new_n196));
  norp02aa1n02x5               g101(.a(\b[16] ), .b(\a[17] ), .o1(new_n197));
  oao003aa1n02x5               g102(.a(new_n189), .b(new_n196), .c(new_n197), .carry(new_n198));
  inv000aa1d42x5               g103(.a(new_n198), .o1(new_n199));
  aoai13aa1n04x5               g104(.a(new_n199), .b(new_n195), .c(new_n184), .d(new_n186), .o1(new_n200));
  xorb03aa1n02x5               g105(.a(new_n200), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g106(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor042aa1n02x5               g107(.a(\b[18] ), .b(\a[19] ), .o1(new_n203));
  nanp02aa1n02x5               g108(.a(\b[18] ), .b(\a[19] ), .o1(new_n204));
  nor042aa1n02x5               g109(.a(\b[19] ), .b(\a[20] ), .o1(new_n205));
  nanp02aa1n02x5               g110(.a(\b[19] ), .b(\a[20] ), .o1(new_n206));
  norb02aa1n06x4               g111(.a(new_n206), .b(new_n205), .out0(new_n207));
  aoi112aa1n02x5               g112(.a(new_n203), .b(new_n207), .c(new_n200), .d(new_n204), .o1(new_n208));
  aoai13aa1n03x5               g113(.a(new_n207), .b(new_n203), .c(new_n200), .d(new_n204), .o1(new_n209));
  norb02aa1n03x4               g114(.a(new_n209), .b(new_n208), .out0(\s[20] ));
  nano23aa1n02x4               g115(.a(new_n203), .b(new_n205), .c(new_n206), .d(new_n204), .out0(new_n211));
  nanp02aa1n02x5               g116(.a(new_n194), .b(new_n211), .o1(new_n212));
  aoi112aa1n02x5               g117(.a(\b[18] ), .b(\a[19] ), .c(\a[20] ), .d(\b[19] ), .o1(new_n213));
  norp02aa1n02x5               g118(.a(\b[17] ), .b(\a[18] ), .o1(new_n214));
  aoi112aa1n02x5               g119(.a(\b[16] ), .b(\a[17] ), .c(\a[18] ), .d(\b[17] ), .o1(new_n215));
  norb02aa1n03x4               g120(.a(new_n204), .b(new_n203), .out0(new_n216));
  oai112aa1n04x5               g121(.a(new_n216), .b(new_n207), .c(new_n215), .d(new_n214), .o1(new_n217));
  nona22aa1n12x5               g122(.a(new_n217), .b(new_n213), .c(new_n205), .out0(new_n218));
  inv000aa1d42x5               g123(.a(new_n218), .o1(new_n219));
  aoai13aa1n04x5               g124(.a(new_n219), .b(new_n212), .c(new_n184), .d(new_n186), .o1(new_n220));
  xorb03aa1n02x5               g125(.a(new_n220), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  norp02aa1n02x5               g126(.a(\b[20] ), .b(\a[21] ), .o1(new_n222));
  nanp02aa1n02x5               g127(.a(\b[20] ), .b(\a[21] ), .o1(new_n223));
  norb02aa1n02x5               g128(.a(new_n223), .b(new_n222), .out0(new_n224));
  norp02aa1n02x5               g129(.a(\b[21] ), .b(\a[22] ), .o1(new_n225));
  nanp02aa1n02x5               g130(.a(\b[21] ), .b(\a[22] ), .o1(new_n226));
  norb02aa1n02x5               g131(.a(new_n226), .b(new_n225), .out0(new_n227));
  aoi112aa1n02x5               g132(.a(new_n222), .b(new_n227), .c(new_n220), .d(new_n224), .o1(new_n228));
  aoai13aa1n03x5               g133(.a(new_n227), .b(new_n222), .c(new_n220), .d(new_n223), .o1(new_n229));
  norb02aa1n02x7               g134(.a(new_n229), .b(new_n228), .out0(\s[22] ));
  nano23aa1n02x4               g135(.a(new_n222), .b(new_n225), .c(new_n226), .d(new_n223), .out0(new_n231));
  aoi012aa1n02x5               g136(.a(new_n225), .b(new_n222), .c(new_n226), .o1(new_n232));
  aobi12aa1n02x5               g137(.a(new_n232), .b(new_n218), .c(new_n231), .out0(new_n233));
  nand23aa1n03x5               g138(.a(new_n194), .b(new_n211), .c(new_n231), .o1(new_n234));
  aoai13aa1n04x5               g139(.a(new_n233), .b(new_n234), .c(new_n184), .d(new_n186), .o1(new_n235));
  xorb03aa1n02x5               g140(.a(new_n235), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  nor042aa1n06x5               g141(.a(\b[22] ), .b(\a[23] ), .o1(new_n237));
  nand02aa1n03x5               g142(.a(\b[22] ), .b(\a[23] ), .o1(new_n238));
  nor042aa1n06x5               g143(.a(\b[23] ), .b(\a[24] ), .o1(new_n239));
  inv000aa1d42x5               g144(.a(new_n239), .o1(new_n240));
  nanp02aa1n02x5               g145(.a(\b[23] ), .b(\a[24] ), .o1(new_n241));
  aoi122aa1n02x7               g146(.a(new_n237), .b(new_n240), .c(new_n241), .d(new_n235), .e(new_n238), .o1(new_n242));
  inv000aa1d42x5               g147(.a(new_n237), .o1(new_n243));
  nanb02aa1n12x5               g148(.a(new_n237), .b(new_n238), .out0(new_n244));
  inv000aa1d42x5               g149(.a(new_n244), .o1(new_n245));
  nand02aa1n02x5               g150(.a(new_n235), .b(new_n245), .o1(new_n246));
  nanb02aa1n02x5               g151(.a(new_n239), .b(new_n241), .out0(new_n247));
  tech160nm_fiaoi012aa1n04x5   g152(.a(new_n247), .b(new_n246), .c(new_n243), .o1(new_n248));
  nor002aa1n02x5               g153(.a(new_n248), .b(new_n242), .o1(\s[24] ));
  nona23aa1n09x5               g154(.a(new_n241), .b(new_n238), .c(new_n237), .d(new_n239), .out0(new_n250));
  inv000aa1n02x5               g155(.a(new_n250), .o1(new_n251));
  nanb03aa1n02x5               g156(.a(new_n212), .b(new_n251), .c(new_n231), .out0(new_n252));
  nanp02aa1n02x5               g157(.a(new_n237), .b(new_n241), .o1(new_n253));
  oai112aa1n04x5               g158(.a(new_n253), .b(new_n240), .c(new_n250), .d(new_n232), .o1(new_n254));
  nano22aa1n03x7               g159(.a(new_n250), .b(new_n224), .c(new_n227), .out0(new_n255));
  aoi012aa1n02x5               g160(.a(new_n254), .b(new_n218), .c(new_n255), .o1(new_n256));
  aoai13aa1n04x5               g161(.a(new_n256), .b(new_n252), .c(new_n184), .d(new_n186), .o1(new_n257));
  xorb03aa1n02x5               g162(.a(new_n257), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g163(.a(\b[24] ), .b(\a[25] ), .o1(new_n259));
  xorc02aa1n02x5               g164(.a(\a[25] ), .b(\b[24] ), .out0(new_n260));
  xorc02aa1n02x5               g165(.a(\a[26] ), .b(\b[25] ), .out0(new_n261));
  aoi112aa1n02x5               g166(.a(new_n259), .b(new_n261), .c(new_n257), .d(new_n260), .o1(new_n262));
  aoai13aa1n03x5               g167(.a(new_n261), .b(new_n259), .c(new_n257), .d(new_n260), .o1(new_n263));
  norb02aa1n02x7               g168(.a(new_n263), .b(new_n262), .out0(\s[26] ));
  inv000aa1d42x5               g169(.a(new_n183), .o1(new_n265));
  inv000aa1d42x5               g170(.a(new_n179), .o1(new_n266));
  nona22aa1n02x4               g171(.a(new_n154), .b(new_n166), .c(new_n266), .out0(new_n267));
  nanp02aa1n02x5               g172(.a(new_n267), .b(new_n265), .o1(new_n268));
  nano32aa1n03x7               g173(.a(new_n234), .b(new_n261), .c(new_n251), .d(new_n260), .out0(new_n269));
  aoai13aa1n06x5               g174(.a(new_n269), .b(new_n268), .c(new_n122), .d(new_n185), .o1(new_n270));
  nor002aa1n02x5               g175(.a(\b[25] ), .b(\a[26] ), .o1(new_n271));
  inv000aa1d42x5               g176(.a(new_n271), .o1(new_n272));
  aoi112aa1n02x7               g177(.a(\b[24] ), .b(\a[25] ), .c(\a[26] ), .d(\b[25] ), .o1(new_n273));
  inv000aa1n02x5               g178(.a(new_n273), .o1(new_n274));
  norp03aa1n02x5               g179(.a(new_n232), .b(new_n244), .c(new_n247), .o1(new_n275));
  nano22aa1n03x5               g180(.a(new_n275), .b(new_n240), .c(new_n253), .out0(new_n276));
  nand02aa1n03x5               g181(.a(new_n218), .b(new_n255), .o1(new_n277));
  and002aa1n09x5               g182(.a(new_n261), .b(new_n260), .o(new_n278));
  inv000aa1d42x5               g183(.a(new_n278), .o1(new_n279));
  aoi012aa1n02x7               g184(.a(new_n279), .b(new_n277), .c(new_n276), .o1(new_n280));
  nano22aa1n03x7               g185(.a(new_n280), .b(new_n272), .c(new_n274), .out0(new_n281));
  xnrc02aa1n02x5               g186(.a(\b[26] ), .b(\a[27] ), .out0(new_n282));
  xobna2aa1n03x5               g187(.a(new_n282), .b(new_n270), .c(new_n281), .out0(\s[27] ));
  nor042aa1n03x5               g188(.a(\b[26] ), .b(\a[27] ), .o1(new_n284));
  inv000aa1d42x5               g189(.a(new_n284), .o1(new_n285));
  xnrc02aa1n02x5               g190(.a(\b[27] ), .b(\a[28] ), .out0(new_n286));
  aoai13aa1n04x5               g191(.a(new_n278), .b(new_n254), .c(new_n218), .d(new_n255), .o1(new_n287));
  nona22aa1n09x5               g192(.a(new_n287), .b(new_n273), .c(new_n271), .out0(new_n288));
  and002aa1n02x5               g193(.a(\b[26] ), .b(\a[27] ), .o(new_n289));
  inv000aa1d42x5               g194(.a(new_n289), .o1(new_n290));
  aoai13aa1n03x5               g195(.a(new_n290), .b(new_n288), .c(new_n187), .d(new_n269), .o1(new_n291));
  tech160nm_fiaoi012aa1n02p5x5 g196(.a(new_n286), .b(new_n291), .c(new_n285), .o1(new_n292));
  tech160nm_fiaoi012aa1n02p5x5 g197(.a(new_n289), .b(new_n270), .c(new_n281), .o1(new_n293));
  nano22aa1n03x5               g198(.a(new_n293), .b(new_n285), .c(new_n286), .out0(new_n294));
  norp02aa1n03x5               g199(.a(new_n292), .b(new_n294), .o1(\s[28] ));
  nor042aa1n03x5               g200(.a(new_n286), .b(new_n282), .o1(new_n296));
  aoai13aa1n03x5               g201(.a(new_n296), .b(new_n288), .c(new_n187), .d(new_n269), .o1(new_n297));
  oao003aa1n02x5               g202(.a(\a[28] ), .b(\b[27] ), .c(new_n285), .carry(new_n298));
  xnrc02aa1n12x5               g203(.a(\b[28] ), .b(\a[29] ), .out0(new_n299));
  tech160nm_fiaoi012aa1n02p5x5 g204(.a(new_n299), .b(new_n297), .c(new_n298), .o1(new_n300));
  inv000aa1d42x5               g205(.a(new_n296), .o1(new_n301));
  tech160nm_fiaoi012aa1n02p5x5 g206(.a(new_n301), .b(new_n270), .c(new_n281), .o1(new_n302));
  nano22aa1n03x5               g207(.a(new_n302), .b(new_n298), .c(new_n299), .out0(new_n303));
  norp02aa1n03x5               g208(.a(new_n300), .b(new_n303), .o1(\s[29] ));
  xorb03aa1n02x5               g209(.a(new_n115), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nor043aa1n03x5               g210(.a(new_n299), .b(new_n286), .c(new_n282), .o1(new_n306));
  aoai13aa1n06x5               g211(.a(new_n306), .b(new_n288), .c(new_n187), .d(new_n269), .o1(new_n307));
  oao003aa1n02x5               g212(.a(\a[29] ), .b(\b[28] ), .c(new_n298), .carry(new_n308));
  xnrc02aa1n02x5               g213(.a(\b[29] ), .b(\a[30] ), .out0(new_n309));
  tech160nm_fiaoi012aa1n05x5   g214(.a(new_n309), .b(new_n307), .c(new_n308), .o1(new_n310));
  inv000aa1d42x5               g215(.a(new_n306), .o1(new_n311));
  tech160nm_fiaoi012aa1n02p5x5 g216(.a(new_n311), .b(new_n270), .c(new_n281), .o1(new_n312));
  nano22aa1n02x4               g217(.a(new_n312), .b(new_n308), .c(new_n309), .out0(new_n313));
  norp02aa1n03x5               g218(.a(new_n310), .b(new_n313), .o1(\s[30] ));
  norb03aa1d15x5               g219(.a(new_n296), .b(new_n309), .c(new_n299), .out0(new_n315));
  inv000aa1d42x5               g220(.a(new_n315), .o1(new_n316));
  tech160nm_fiaoi012aa1n02p5x5 g221(.a(new_n316), .b(new_n270), .c(new_n281), .o1(new_n317));
  oao003aa1n02x5               g222(.a(\a[30] ), .b(\b[29] ), .c(new_n308), .carry(new_n318));
  xnrc02aa1n02x5               g223(.a(\b[30] ), .b(\a[31] ), .out0(new_n319));
  nano22aa1n03x5               g224(.a(new_n317), .b(new_n318), .c(new_n319), .out0(new_n320));
  aoai13aa1n03x5               g225(.a(new_n315), .b(new_n288), .c(new_n187), .d(new_n269), .o1(new_n321));
  tech160nm_fiaoi012aa1n02p5x5 g226(.a(new_n319), .b(new_n321), .c(new_n318), .o1(new_n322));
  norp02aa1n03x5               g227(.a(new_n322), .b(new_n320), .o1(\s[31] ));
  xnrb03aa1n02x5               g228(.a(new_n128), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g229(.a(\a[3] ), .b(\b[2] ), .c(new_n128), .o1(new_n325));
  xorb03aa1n02x5               g230(.a(new_n325), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g231(.a(new_n129), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oaoi03aa1n02x5               g232(.a(\a[5] ), .b(\b[4] ), .c(new_n118), .o1(new_n328));
  xorb03aa1n02x5               g233(.a(new_n328), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  aoai13aa1n02x5               g234(.a(new_n101), .b(new_n102), .c(new_n328), .d(new_n103), .o1(new_n330));
  aoi112aa1n02x5               g235(.a(new_n101), .b(new_n102), .c(new_n328), .d(new_n103), .o1(new_n331));
  norb02aa1n02x5               g236(.a(new_n330), .b(new_n331), .out0(\s[7] ));
  orn002aa1n02x5               g237(.a(\a[7] ), .b(\b[6] ), .o(new_n333));
  xnbna2aa1n03x5               g238(.a(new_n100), .b(new_n330), .c(new_n333), .out0(\s[8] ));
  xorb03aa1n02x5               g239(.a(new_n122), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


