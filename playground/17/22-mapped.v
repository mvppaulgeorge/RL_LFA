// Benchmark "adder" written by ABC on Wed Jul 17 20:51:14 2024

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
    new_n133, new_n134, new_n135, new_n136, new_n138, new_n139, new_n140,
    new_n141, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n158, new_n159, new_n160, new_n162, new_n163, new_n164,
    new_n165, new_n166, new_n167, new_n168, new_n169, new_n170, new_n171,
    new_n173, new_n174, new_n175, new_n176, new_n177, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n187, new_n188,
    new_n189, new_n190, new_n191, new_n192, new_n193, new_n194, new_n195,
    new_n196, new_n197, new_n198, new_n199, new_n200, new_n202, new_n203,
    new_n204, new_n205, new_n206, new_n209, new_n210, new_n211, new_n212,
    new_n213, new_n214, new_n215, new_n216, new_n217, new_n219, new_n220,
    new_n221, new_n222, new_n223, new_n224, new_n226, new_n227, new_n228,
    new_n229, new_n230, new_n231, new_n232, new_n234, new_n235, new_n236,
    new_n237, new_n238, new_n239, new_n240, new_n241, new_n242, new_n244,
    new_n245, new_n246, new_n247, new_n248, new_n249, new_n251, new_n252,
    new_n253, new_n254, new_n255, new_n256, new_n257, new_n258, new_n259,
    new_n260, new_n262, new_n263, new_n264, new_n265, new_n266, new_n267,
    new_n269, new_n270, new_n271, new_n272, new_n273, new_n274, new_n275,
    new_n276, new_n277, new_n279, new_n280, new_n281, new_n282, new_n283,
    new_n284, new_n285, new_n286, new_n287, new_n288, new_n289, new_n290,
    new_n291, new_n292, new_n293, new_n295, new_n296, new_n297, new_n298,
    new_n299, new_n300, new_n301, new_n302, new_n305, new_n306, new_n307,
    new_n308, new_n309, new_n310, new_n311, new_n312, new_n313, new_n314,
    new_n316, new_n317, new_n318, new_n319, new_n320, new_n321, new_n322,
    new_n323, new_n324, new_n327, new_n328, new_n331, new_n333, new_n334,
    new_n335, new_n336, new_n338;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor042aa1n02x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  tech160nm_finand02aa1n05x5   g002(.a(\b[3] ), .b(\a[4] ), .o1(new_n98));
  nand42aa1n02x5               g003(.a(\b[0] ), .b(\a[1] ), .o1(new_n99));
  nand42aa1n03x5               g004(.a(\b[1] ), .b(\a[2] ), .o1(new_n100));
  nor042aa1n02x5               g005(.a(\b[1] ), .b(\a[2] ), .o1(new_n101));
  nona22aa1n06x5               g006(.a(new_n100), .b(new_n101), .c(new_n99), .out0(new_n102));
  norp02aa1n04x5               g007(.a(\b[2] ), .b(\a[3] ), .o1(new_n103));
  nand42aa1n06x5               g008(.a(\b[2] ), .b(\a[3] ), .o1(new_n104));
  nano22aa1n02x5               g009(.a(new_n103), .b(new_n100), .c(new_n104), .out0(new_n105));
  orn002aa1n24x5               g010(.a(\a[4] ), .b(\b[3] ), .o(new_n106));
  oai112aa1n06x5               g011(.a(new_n106), .b(new_n98), .c(\b[2] ), .d(\a[3] ), .o1(new_n107));
  aoai13aa1n04x5               g012(.a(new_n98), .b(new_n107), .c(new_n105), .d(new_n102), .o1(new_n108));
  nor002aa1n03x5               g013(.a(\b[7] ), .b(\a[8] ), .o1(new_n109));
  nand02aa1n06x5               g014(.a(\b[7] ), .b(\a[8] ), .o1(new_n110));
  nor002aa1d32x5               g015(.a(\b[6] ), .b(\a[7] ), .o1(new_n111));
  nand42aa1n04x5               g016(.a(\b[6] ), .b(\a[7] ), .o1(new_n112));
  nano23aa1n09x5               g017(.a(new_n109), .b(new_n111), .c(new_n112), .d(new_n110), .out0(new_n113));
  tech160nm_fixorc02aa1n03p5x5 g018(.a(\a[6] ), .b(\b[5] ), .out0(new_n114));
  xorc02aa1n12x5               g019(.a(\a[5] ), .b(\b[4] ), .out0(new_n115));
  nand23aa1n06x5               g020(.a(new_n113), .b(new_n114), .c(new_n115), .o1(new_n116));
  inv000aa1d42x5               g021(.a(new_n111), .o1(new_n117));
  oaoi03aa1n12x5               g022(.a(\a[8] ), .b(\b[7] ), .c(new_n117), .o1(new_n118));
  inv000aa1d42x5               g023(.a(\a[6] ), .o1(new_n119));
  inv000aa1d42x5               g024(.a(\b[5] ), .o1(new_n120));
  nor042aa1n02x5               g025(.a(\b[4] ), .b(\a[5] ), .o1(new_n121));
  oao003aa1n06x5               g026(.a(new_n119), .b(new_n120), .c(new_n121), .carry(new_n122));
  aoi012aa1n06x5               g027(.a(new_n118), .b(new_n113), .c(new_n122), .o1(new_n123));
  tech160nm_fioai012aa1n05x5   g028(.a(new_n123), .b(new_n108), .c(new_n116), .o1(new_n124));
  nanp02aa1n12x5               g029(.a(\b[8] ), .b(\a[9] ), .o1(new_n125));
  aoi012aa1n02x5               g030(.a(new_n97), .b(new_n124), .c(new_n125), .o1(new_n126));
  xnrb03aa1n02x5               g031(.a(new_n126), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  nor042aa1n09x5               g032(.a(\b[9] ), .b(\a[10] ), .o1(new_n128));
  nand42aa1n06x5               g033(.a(\b[9] ), .b(\a[10] ), .o1(new_n129));
  nano23aa1n03x7               g034(.a(new_n97), .b(new_n128), .c(new_n129), .d(new_n125), .out0(new_n130));
  tech160nm_fiao0012aa1n02p5x5 g035(.a(new_n128), .b(new_n97), .c(new_n129), .o(new_n131));
  nor002aa1d32x5               g036(.a(\b[10] ), .b(\a[11] ), .o1(new_n132));
  nand02aa1n06x5               g037(.a(\b[10] ), .b(\a[11] ), .o1(new_n133));
  norb02aa1n12x5               g038(.a(new_n133), .b(new_n132), .out0(new_n134));
  aoai13aa1n02x5               g039(.a(new_n134), .b(new_n131), .c(new_n124), .d(new_n130), .o1(new_n135));
  aoi112aa1n02x5               g040(.a(new_n134), .b(new_n131), .c(new_n124), .d(new_n130), .o1(new_n136));
  norb02aa1n02x5               g041(.a(new_n135), .b(new_n136), .out0(\s[11] ));
  inv000aa1d42x5               g042(.a(new_n132), .o1(new_n138));
  nor042aa1d18x5               g043(.a(\b[11] ), .b(\a[12] ), .o1(new_n139));
  nand02aa1d28x5               g044(.a(\b[11] ), .b(\a[12] ), .o1(new_n140));
  norb02aa1n12x5               g045(.a(new_n140), .b(new_n139), .out0(new_n141));
  xnbna2aa1n03x5               g046(.a(new_n141), .b(new_n135), .c(new_n138), .out0(\s[12] ));
  norb02aa1n06x5               g047(.a(new_n104), .b(new_n103), .out0(new_n143));
  oai112aa1n04x5               g048(.a(new_n143), .b(new_n100), .c(new_n101), .d(new_n99), .o1(new_n144));
  inv000aa1n02x5               g049(.a(new_n107), .o1(new_n145));
  aobi12aa1n12x5               g050(.a(new_n98), .b(new_n144), .c(new_n145), .out0(new_n146));
  inv040aa1n02x5               g051(.a(new_n116), .o1(new_n147));
  inv030aa1n02x5               g052(.a(new_n123), .o1(new_n148));
  aoi012aa1n02x5               g053(.a(new_n148), .b(new_n146), .c(new_n147), .o1(new_n149));
  nano23aa1n06x5               g054(.a(new_n132), .b(new_n139), .c(new_n140), .d(new_n133), .out0(new_n150));
  nanp02aa1n02x5               g055(.a(new_n150), .b(new_n130), .o1(new_n151));
  aoi112aa1n09x5               g056(.a(\b[8] ), .b(\a[9] ), .c(\a[10] ), .d(\b[9] ), .o1(new_n152));
  oai112aa1n06x5               g057(.a(new_n134), .b(new_n141), .c(new_n152), .d(new_n128), .o1(new_n153));
  aoi012aa1d18x5               g058(.a(new_n139), .b(new_n132), .c(new_n140), .o1(new_n154));
  nanp02aa1n02x5               g059(.a(new_n153), .b(new_n154), .o1(new_n155));
  oabi12aa1n02x5               g060(.a(new_n155), .b(new_n149), .c(new_n151), .out0(new_n156));
  xorb03aa1n02x5               g061(.a(new_n156), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  nor022aa1n16x5               g062(.a(\b[12] ), .b(\a[13] ), .o1(new_n158));
  nanp02aa1n04x5               g063(.a(\b[12] ), .b(\a[13] ), .o1(new_n159));
  aoi012aa1n02x5               g064(.a(new_n158), .b(new_n156), .c(new_n159), .o1(new_n160));
  xnrb03aa1n02x5               g065(.a(new_n160), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  nor002aa1d32x5               g066(.a(\b[13] ), .b(\a[14] ), .o1(new_n162));
  nand02aa1n06x5               g067(.a(\b[13] ), .b(\a[14] ), .o1(new_n163));
  nona23aa1d16x5               g068(.a(new_n163), .b(new_n159), .c(new_n158), .d(new_n162), .out0(new_n164));
  tech160nm_fiaoi012aa1n03p5x5 g069(.a(new_n162), .b(new_n158), .c(new_n163), .o1(new_n165));
  aoai13aa1n12x5               g070(.a(new_n165), .b(new_n164), .c(new_n153), .d(new_n154), .o1(new_n166));
  inv000aa1d42x5               g071(.a(new_n166), .o1(new_n167));
  nona22aa1n03x5               g072(.a(new_n124), .b(new_n151), .c(new_n164), .out0(new_n168));
  nor022aa1n16x5               g073(.a(\b[14] ), .b(\a[15] ), .o1(new_n169));
  nand42aa1n08x5               g074(.a(\b[14] ), .b(\a[15] ), .o1(new_n170));
  norb02aa1n02x5               g075(.a(new_n170), .b(new_n169), .out0(new_n171));
  xnbna2aa1n03x5               g076(.a(new_n171), .b(new_n168), .c(new_n167), .out0(\s[15] ));
  inv000aa1d42x5               g077(.a(new_n169), .o1(new_n173));
  aob012aa1n03x5               g078(.a(new_n171), .b(new_n168), .c(new_n167), .out0(new_n174));
  nor042aa1n04x5               g079(.a(\b[15] ), .b(\a[16] ), .o1(new_n175));
  nand42aa1n08x5               g080(.a(\b[15] ), .b(\a[16] ), .o1(new_n176));
  norb02aa1n02x5               g081(.a(new_n176), .b(new_n175), .out0(new_n177));
  xnbna2aa1n03x5               g082(.a(new_n177), .b(new_n174), .c(new_n173), .out0(\s[16] ));
  nano23aa1d15x5               g083(.a(new_n169), .b(new_n175), .c(new_n176), .d(new_n170), .out0(new_n179));
  inv000aa1n03x5               g084(.a(new_n179), .o1(new_n180));
  nor043aa1n03x5               g085(.a(new_n151), .b(new_n164), .c(new_n180), .o1(new_n181));
  aoai13aa1n12x5               g086(.a(new_n181), .b(new_n148), .c(new_n146), .d(new_n147), .o1(new_n182));
  aoi012aa1n02x5               g087(.a(new_n175), .b(new_n169), .c(new_n176), .o1(new_n183));
  aobi12aa1n18x5               g088(.a(new_n183), .b(new_n166), .c(new_n179), .out0(new_n184));
  xorc02aa1n02x5               g089(.a(\a[17] ), .b(\b[16] ), .out0(new_n185));
  xnbna2aa1n03x5               g090(.a(new_n185), .b(new_n184), .c(new_n182), .out0(\s[17] ));
  inv000aa1d42x5               g091(.a(\a[17] ), .o1(new_n187));
  inv000aa1d42x5               g092(.a(\b[16] ), .o1(new_n188));
  nanp02aa1n02x5               g093(.a(new_n188), .b(new_n187), .o1(new_n189));
  nona23aa1n02x4               g094(.a(new_n129), .b(new_n125), .c(new_n97), .d(new_n128), .out0(new_n190));
  nona23aa1n03x5               g095(.a(new_n179), .b(new_n150), .c(new_n190), .d(new_n164), .out0(new_n191));
  oaoi13aa1n06x5               g096(.a(new_n191), .b(new_n123), .c(new_n108), .d(new_n116), .o1(new_n192));
  inv000aa1d42x5               g097(.a(new_n154), .o1(new_n193));
  inv000aa1d42x5               g098(.a(new_n164), .o1(new_n194));
  aoai13aa1n03x5               g099(.a(new_n194), .b(new_n193), .c(new_n150), .d(new_n131), .o1(new_n195));
  aoai13aa1n02x5               g100(.a(new_n183), .b(new_n180), .c(new_n195), .d(new_n165), .o1(new_n196));
  oai012aa1n02x5               g101(.a(new_n185), .b(new_n196), .c(new_n192), .o1(new_n197));
  nor042aa1n03x5               g102(.a(\b[17] ), .b(\a[18] ), .o1(new_n198));
  nanp02aa1n03x5               g103(.a(\b[17] ), .b(\a[18] ), .o1(new_n199));
  nanb02aa1n02x5               g104(.a(new_n198), .b(new_n199), .out0(new_n200));
  xobna2aa1n03x5               g105(.a(new_n200), .b(new_n197), .c(new_n189), .out0(\s[18] ));
  inv040aa1d32x5               g106(.a(\a[18] ), .o1(new_n202));
  xroi22aa1d06x4               g107(.a(new_n187), .b(\b[16] ), .c(new_n202), .d(\b[17] ), .out0(new_n203));
  inv000aa1d42x5               g108(.a(new_n203), .o1(new_n204));
  aoi013aa1n06x5               g109(.a(new_n198), .b(new_n199), .c(new_n187), .d(new_n188), .o1(new_n205));
  aoai13aa1n06x5               g110(.a(new_n205), .b(new_n204), .c(new_n184), .d(new_n182), .o1(new_n206));
  xorb03aa1n03x5               g111(.a(new_n206), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g112(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor042aa1n06x5               g113(.a(\b[18] ), .b(\a[19] ), .o1(new_n209));
  nanp02aa1n04x5               g114(.a(\b[18] ), .b(\a[19] ), .o1(new_n210));
  norb02aa1n02x7               g115(.a(new_n210), .b(new_n209), .out0(new_n211));
  nor022aa1n16x5               g116(.a(\b[19] ), .b(\a[20] ), .o1(new_n212));
  nand22aa1n06x5               g117(.a(\b[19] ), .b(\a[20] ), .o1(new_n213));
  nanb02aa1n02x5               g118(.a(new_n212), .b(new_n213), .out0(new_n214));
  aoai13aa1n03x5               g119(.a(new_n214), .b(new_n209), .c(new_n206), .d(new_n211), .o1(new_n215));
  nand02aa1n04x5               g120(.a(new_n206), .b(new_n211), .o1(new_n216));
  nona22aa1n02x4               g121(.a(new_n216), .b(new_n214), .c(new_n209), .out0(new_n217));
  nanp02aa1n03x5               g122(.a(new_n217), .b(new_n215), .o1(\s[20] ));
  nona23aa1n02x4               g123(.a(new_n211), .b(new_n185), .c(new_n214), .d(new_n200), .out0(new_n219));
  nona23aa1n09x5               g124(.a(new_n213), .b(new_n210), .c(new_n209), .d(new_n212), .out0(new_n220));
  ao0012aa1n12x5               g125(.a(new_n212), .b(new_n209), .c(new_n213), .o(new_n221));
  oabi12aa1n18x5               g126(.a(new_n221), .b(new_n220), .c(new_n205), .out0(new_n222));
  inv000aa1d42x5               g127(.a(new_n222), .o1(new_n223));
  aoai13aa1n06x5               g128(.a(new_n223), .b(new_n219), .c(new_n184), .d(new_n182), .o1(new_n224));
  xorb03aa1n02x5               g129(.a(new_n224), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor042aa1n02x5               g130(.a(\b[20] ), .b(\a[21] ), .o1(new_n226));
  xnrc02aa1n12x5               g131(.a(\b[20] ), .b(\a[21] ), .out0(new_n227));
  inv000aa1d42x5               g132(.a(new_n227), .o1(new_n228));
  xnrc02aa1n12x5               g133(.a(\b[21] ), .b(\a[22] ), .out0(new_n229));
  aoai13aa1n03x5               g134(.a(new_n229), .b(new_n226), .c(new_n224), .d(new_n228), .o1(new_n230));
  nand42aa1n02x5               g135(.a(new_n224), .b(new_n228), .o1(new_n231));
  nona22aa1n03x5               g136(.a(new_n231), .b(new_n229), .c(new_n226), .out0(new_n232));
  nanp02aa1n03x5               g137(.a(new_n232), .b(new_n230), .o1(\s[22] ));
  nano23aa1n09x5               g138(.a(new_n209), .b(new_n212), .c(new_n213), .d(new_n210), .out0(new_n234));
  nor042aa1n06x5               g139(.a(new_n229), .b(new_n227), .o1(new_n235));
  nand23aa1n03x5               g140(.a(new_n203), .b(new_n235), .c(new_n234), .o1(new_n236));
  inv000aa1d42x5               g141(.a(\a[22] ), .o1(new_n237));
  inv000aa1d42x5               g142(.a(\b[21] ), .o1(new_n238));
  oaoi03aa1n09x5               g143(.a(new_n237), .b(new_n238), .c(new_n226), .o1(new_n239));
  inv000aa1d42x5               g144(.a(new_n239), .o1(new_n240));
  aoi012aa1n02x5               g145(.a(new_n240), .b(new_n222), .c(new_n235), .o1(new_n241));
  aoai13aa1n06x5               g146(.a(new_n241), .b(new_n236), .c(new_n184), .d(new_n182), .o1(new_n242));
  xorb03aa1n02x5               g147(.a(new_n242), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g148(.a(\b[22] ), .b(\a[23] ), .o1(new_n244));
  xorc02aa1n12x5               g149(.a(\a[23] ), .b(\b[22] ), .out0(new_n245));
  xnrc02aa1n12x5               g150(.a(\b[23] ), .b(\a[24] ), .out0(new_n246));
  aoai13aa1n03x5               g151(.a(new_n246), .b(new_n244), .c(new_n242), .d(new_n245), .o1(new_n247));
  nand42aa1n02x5               g152(.a(new_n242), .b(new_n245), .o1(new_n248));
  nona22aa1n03x5               g153(.a(new_n248), .b(new_n246), .c(new_n244), .out0(new_n249));
  nanp02aa1n03x5               g154(.a(new_n249), .b(new_n247), .o1(\s[24] ));
  norb02aa1n06x5               g155(.a(new_n245), .b(new_n246), .out0(new_n251));
  nanb03aa1n02x5               g156(.a(new_n219), .b(new_n251), .c(new_n235), .out0(new_n252));
  oaoi03aa1n02x5               g157(.a(\a[18] ), .b(\b[17] ), .c(new_n189), .o1(new_n253));
  aoai13aa1n06x5               g158(.a(new_n235), .b(new_n221), .c(new_n234), .d(new_n253), .o1(new_n254));
  inv040aa1d28x5               g159(.a(new_n251), .o1(new_n255));
  orn002aa1n02x5               g160(.a(\a[23] ), .b(\b[22] ), .o(new_n256));
  oao003aa1n02x5               g161(.a(\a[24] ), .b(\b[23] ), .c(new_n256), .carry(new_n257));
  aoai13aa1n12x5               g162(.a(new_n257), .b(new_n255), .c(new_n254), .d(new_n239), .o1(new_n258));
  inv000aa1d42x5               g163(.a(new_n258), .o1(new_n259));
  aoai13aa1n06x5               g164(.a(new_n259), .b(new_n252), .c(new_n184), .d(new_n182), .o1(new_n260));
  xorb03aa1n02x5               g165(.a(new_n260), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g166(.a(\b[24] ), .b(\a[25] ), .o1(new_n262));
  xorc02aa1n06x5               g167(.a(\a[25] ), .b(\b[24] ), .out0(new_n263));
  xnrc02aa1n12x5               g168(.a(\b[25] ), .b(\a[26] ), .out0(new_n264));
  aoai13aa1n03x5               g169(.a(new_n264), .b(new_n262), .c(new_n260), .d(new_n263), .o1(new_n265));
  nand42aa1n02x5               g170(.a(new_n260), .b(new_n263), .o1(new_n266));
  nona22aa1n03x5               g171(.a(new_n266), .b(new_n264), .c(new_n262), .out0(new_n267));
  nanp02aa1n03x5               g172(.a(new_n267), .b(new_n265), .o1(\s[26] ));
  norb02aa1n02x5               g173(.a(new_n263), .b(new_n264), .out0(new_n269));
  nano22aa1n03x7               g174(.a(new_n236), .b(new_n251), .c(new_n269), .out0(new_n270));
  tech160nm_fioai012aa1n05x5   g175(.a(new_n270), .b(new_n196), .c(new_n192), .o1(new_n271));
  inv000aa1d42x5               g176(.a(\a[26] ), .o1(new_n272));
  inv000aa1d42x5               g177(.a(\b[25] ), .o1(new_n273));
  tech160nm_fioaoi03aa1n04x5   g178(.a(new_n272), .b(new_n273), .c(new_n262), .o1(new_n274));
  inv000aa1d42x5               g179(.a(new_n274), .o1(new_n275));
  aoi012aa1n12x5               g180(.a(new_n275), .b(new_n258), .c(new_n269), .o1(new_n276));
  tech160nm_fixorc02aa1n03p5x5 g181(.a(\a[27] ), .b(\b[26] ), .out0(new_n277));
  xnbna2aa1n03x5               g182(.a(new_n277), .b(new_n271), .c(new_n276), .out0(\s[27] ));
  norp02aa1n02x5               g183(.a(\b[26] ), .b(\a[27] ), .o1(new_n279));
  inv000aa1n03x5               g184(.a(new_n279), .o1(new_n280));
  inv000aa1d42x5               g185(.a(new_n277), .o1(new_n281));
  aoai13aa1n06x5               g186(.a(new_n280), .b(new_n281), .c(new_n271), .d(new_n276), .o1(new_n282));
  norp02aa1n02x5               g187(.a(\b[27] ), .b(\a[28] ), .o1(new_n283));
  nanp02aa1n02x5               g188(.a(\b[27] ), .b(\a[28] ), .o1(new_n284));
  norb02aa1n03x5               g189(.a(new_n284), .b(new_n283), .out0(new_n285));
  inv000aa1d42x5               g190(.a(new_n285), .o1(new_n286));
  nanp02aa1n03x5               g191(.a(new_n282), .b(new_n286), .o1(new_n287));
  nanp02aa1n09x5               g192(.a(new_n184), .b(new_n182), .o1(new_n288));
  aoai13aa1n06x5               g193(.a(new_n251), .b(new_n240), .c(new_n222), .d(new_n235), .o1(new_n289));
  inv000aa1n02x5               g194(.a(new_n269), .o1(new_n290));
  aoai13aa1n12x5               g195(.a(new_n274), .b(new_n290), .c(new_n289), .d(new_n257), .o1(new_n291));
  aoai13aa1n03x5               g196(.a(new_n277), .b(new_n291), .c(new_n288), .d(new_n270), .o1(new_n292));
  nona22aa1n03x5               g197(.a(new_n292), .b(new_n286), .c(new_n279), .out0(new_n293));
  nanp02aa1n03x5               g198(.a(new_n287), .b(new_n293), .o1(\s[28] ));
  tech160nm_fixorc02aa1n03p5x5 g199(.a(\a[29] ), .b(\b[28] ), .out0(new_n295));
  norb02aa1n02x5               g200(.a(new_n277), .b(new_n286), .out0(new_n296));
  aoai13aa1n03x5               g201(.a(new_n296), .b(new_n291), .c(new_n288), .d(new_n270), .o1(new_n297));
  inv000aa1d42x5               g202(.a(new_n295), .o1(new_n298));
  oaoi03aa1n09x5               g203(.a(\a[28] ), .b(\b[27] ), .c(new_n280), .o1(new_n299));
  nona22aa1n03x5               g204(.a(new_n297), .b(new_n298), .c(new_n299), .out0(new_n300));
  aobi12aa1n06x5               g205(.a(new_n270), .b(new_n184), .c(new_n182), .out0(new_n301));
  oaoi13aa1n02x7               g206(.a(new_n299), .b(new_n296), .c(new_n301), .d(new_n291), .o1(new_n302));
  oai012aa1n03x5               g207(.a(new_n300), .b(new_n302), .c(new_n295), .o1(\s[29] ));
  xorb03aa1n02x5               g208(.a(new_n99), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n02x4               g209(.a(new_n298), .b(new_n277), .c(new_n285), .out0(new_n305));
  inv000aa1d42x5               g210(.a(\a[29] ), .o1(new_n306));
  inv000aa1d42x5               g211(.a(\b[28] ), .o1(new_n307));
  oaoi03aa1n09x5               g212(.a(new_n306), .b(new_n307), .c(new_n299), .o1(new_n308));
  inv000aa1d42x5               g213(.a(new_n308), .o1(new_n309));
  oaoi13aa1n02x7               g214(.a(new_n309), .b(new_n305), .c(new_n301), .d(new_n291), .o1(new_n310));
  xorc02aa1n02x5               g215(.a(\a[30] ), .b(\b[29] ), .out0(new_n311));
  aoai13aa1n03x5               g216(.a(new_n305), .b(new_n291), .c(new_n288), .d(new_n270), .o1(new_n312));
  inv000aa1d42x5               g217(.a(new_n311), .o1(new_n313));
  nona22aa1n03x5               g218(.a(new_n312), .b(new_n309), .c(new_n313), .out0(new_n314));
  oai012aa1n03x5               g219(.a(new_n314), .b(new_n310), .c(new_n311), .o1(\s[30] ));
  nano32aa1n03x7               g220(.a(new_n313), .b(new_n295), .c(new_n277), .d(new_n285), .out0(new_n316));
  inv020aa1n02x5               g221(.a(new_n316), .o1(new_n317));
  oaoi03aa1n02x5               g222(.a(\a[30] ), .b(\b[29] ), .c(new_n308), .o1(new_n318));
  inv000aa1n02x5               g223(.a(new_n318), .o1(new_n319));
  aoai13aa1n06x5               g224(.a(new_n319), .b(new_n317), .c(new_n271), .d(new_n276), .o1(new_n320));
  xnrc02aa1n02x5               g225(.a(\b[30] ), .b(\a[31] ), .out0(new_n321));
  nand42aa1n02x5               g226(.a(new_n320), .b(new_n321), .o1(new_n322));
  aoai13aa1n02x7               g227(.a(new_n316), .b(new_n291), .c(new_n288), .d(new_n270), .o1(new_n323));
  nona22aa1n03x5               g228(.a(new_n323), .b(new_n318), .c(new_n321), .out0(new_n324));
  nanp02aa1n03x5               g229(.a(new_n322), .b(new_n324), .o1(\s[31] ));
  xobna2aa1n03x5               g230(.a(new_n143), .b(new_n102), .c(new_n100), .out0(\s[3] ));
  nanp02aa1n02x5               g231(.a(new_n144), .b(new_n145), .o1(new_n327));
  aoi012aa1n02x5               g232(.a(new_n103), .b(new_n105), .c(new_n102), .o1(new_n328));
  aoai13aa1n02x5               g233(.a(new_n327), .b(new_n328), .c(new_n106), .d(new_n98), .o1(\s[4] ));
  xnrc02aa1n02x5               g234(.a(new_n108), .b(new_n115), .out0(\s[5] ));
  aoi013aa1n02x4               g235(.a(new_n121), .b(new_n327), .c(new_n98), .d(new_n115), .o1(new_n331));
  xorb03aa1n02x5               g236(.a(new_n331), .b(\b[5] ), .c(new_n119), .out0(\s[6] ));
  norb02aa1n02x5               g237(.a(new_n112), .b(new_n111), .out0(new_n333));
  nanp02aa1n02x5               g238(.a(new_n331), .b(new_n114), .o1(new_n334));
  oai112aa1n03x5               g239(.a(new_n334), .b(new_n333), .c(new_n120), .d(new_n119), .o1(new_n335));
  oaoi13aa1n02x5               g240(.a(new_n333), .b(new_n334), .c(new_n119), .d(new_n120), .o1(new_n336));
  norb02aa1n02x5               g241(.a(new_n335), .b(new_n336), .out0(\s[7] ));
  norb02aa1n02x5               g242(.a(new_n110), .b(new_n109), .out0(new_n338));
  xnbna2aa1n03x5               g243(.a(new_n338), .b(new_n335), .c(new_n117), .out0(\s[8] ));
  xorb03aa1n02x5               g244(.a(new_n124), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


