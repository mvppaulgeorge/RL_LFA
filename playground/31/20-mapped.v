// Benchmark "adder" written by ABC on Thu Jul 18 04:00:30 2024

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
    new_n125, new_n126, new_n127, new_n128, new_n130, new_n131, new_n132,
    new_n133, new_n134, new_n136, new_n138, new_n139, new_n140, new_n141,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n159, new_n160, new_n161, new_n163, new_n164,
    new_n165, new_n166, new_n167, new_n169, new_n170, new_n171, new_n172,
    new_n173, new_n174, new_n175, new_n176, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n189, new_n190, new_n191, new_n192, new_n194, new_n195, new_n196,
    new_n197, new_n198, new_n199, new_n200, new_n201, new_n202, new_n203,
    new_n204, new_n207, new_n208, new_n209, new_n210, new_n211, new_n212,
    new_n213, new_n214, new_n215, new_n216, new_n218, new_n219, new_n220,
    new_n221, new_n222, new_n223, new_n224, new_n225, new_n226, new_n228,
    new_n229, new_n230, new_n231, new_n232, new_n233, new_n234, new_n235,
    new_n236, new_n237, new_n239, new_n240, new_n241, new_n242, new_n243,
    new_n245, new_n246, new_n247, new_n248, new_n249, new_n250, new_n251,
    new_n252, new_n253, new_n254, new_n255, new_n256, new_n258, new_n259,
    new_n260, new_n261, new_n262, new_n263, new_n264, new_n265, new_n266,
    new_n267, new_n269, new_n270, new_n271, new_n272, new_n273, new_n274,
    new_n275, new_n277, new_n278, new_n279, new_n280, new_n281, new_n282,
    new_n283, new_n284, new_n286, new_n287, new_n288, new_n289, new_n290,
    new_n291, new_n292, new_n293, new_n294, new_n295, new_n296, new_n298,
    new_n299, new_n300, new_n301, new_n302, new_n303, new_n304, new_n305,
    new_n308, new_n309, new_n310, new_n311, new_n312, new_n313, new_n314,
    new_n315, new_n317, new_n318, new_n319, new_n320, new_n321, new_n322,
    new_n323, new_n324, new_n327, new_n330, new_n332, new_n333, new_n334,
    new_n335, new_n336, new_n338;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  inv000aa1d42x5               g001(.a(\a[10] ), .o1(new_n97));
  nor042aa1n02x5               g002(.a(\b[8] ), .b(\a[9] ), .o1(new_n98));
  nand42aa1n02x5               g003(.a(\b[7] ), .b(\a[8] ), .o1(new_n99));
  nor042aa1n06x5               g004(.a(\b[7] ), .b(\a[8] ), .o1(new_n100));
  nor002aa1d32x5               g005(.a(\b[6] ), .b(\a[7] ), .o1(new_n101));
  nand22aa1n12x5               g006(.a(\b[6] ), .b(\a[7] ), .o1(new_n102));
  nano23aa1n02x5               g007(.a(new_n101), .b(new_n100), .c(new_n102), .d(new_n99), .out0(new_n103));
  inv000aa1d42x5               g008(.a(\a[6] ), .o1(new_n104));
  inv000aa1d42x5               g009(.a(\b[5] ), .o1(new_n105));
  nor002aa1n02x5               g010(.a(\b[4] ), .b(\a[5] ), .o1(new_n106));
  oao003aa1n02x5               g011(.a(new_n104), .b(new_n105), .c(new_n106), .carry(new_n107));
  inv000aa1d42x5               g012(.a(new_n101), .o1(new_n108));
  oaoi03aa1n02x5               g013(.a(\a[8] ), .b(\b[7] ), .c(new_n108), .o1(new_n109));
  aoi012aa1n02x7               g014(.a(new_n109), .b(new_n103), .c(new_n107), .o1(new_n110));
  xorc02aa1n02x5               g015(.a(\a[4] ), .b(\b[3] ), .out0(new_n111));
  nor002aa1n03x5               g016(.a(\b[2] ), .b(\a[3] ), .o1(new_n112));
  nanp02aa1n02x5               g017(.a(\b[2] ), .b(\a[3] ), .o1(new_n113));
  norb02aa1n02x5               g018(.a(new_n113), .b(new_n112), .out0(new_n114));
  nanp02aa1n02x5               g019(.a(\b[1] ), .b(\a[2] ), .o1(new_n115));
  nand22aa1n04x5               g020(.a(\b[0] ), .b(\a[1] ), .o1(new_n116));
  nor042aa1n02x5               g021(.a(\b[1] ), .b(\a[2] ), .o1(new_n117));
  oai012aa1n12x5               g022(.a(new_n115), .b(new_n117), .c(new_n116), .o1(new_n118));
  nanb03aa1n02x5               g023(.a(new_n118), .b(new_n111), .c(new_n114), .out0(new_n119));
  inv000aa1d42x5               g024(.a(\a[4] ), .o1(new_n120));
  inv000aa1d42x5               g025(.a(\b[3] ), .o1(new_n121));
  tech160nm_fioaoi03aa1n03p5x5 g026(.a(new_n120), .b(new_n121), .c(new_n112), .o1(new_n122));
  tech160nm_fixnrc02aa1n02p5x5 g027(.a(\b[5] ), .b(\a[6] ), .out0(new_n123));
  xnrc02aa1n02x5               g028(.a(\b[4] ), .b(\a[5] ), .out0(new_n124));
  nona22aa1n02x4               g029(.a(new_n103), .b(new_n123), .c(new_n124), .out0(new_n125));
  aoai13aa1n04x5               g030(.a(new_n110), .b(new_n125), .c(new_n119), .d(new_n122), .o1(new_n126));
  xnrc02aa1n02x5               g031(.a(\b[8] ), .b(\a[9] ), .out0(new_n127));
  aoib12aa1n02x5               g032(.a(new_n98), .b(new_n126), .c(new_n127), .out0(new_n128));
  xorb03aa1n02x5               g033(.a(new_n128), .b(\b[9] ), .c(new_n97), .out0(\s[10] ));
  xnrc02aa1n02x5               g034(.a(\b[9] ), .b(\a[10] ), .out0(new_n130));
  norp02aa1n02x5               g035(.a(new_n130), .b(new_n127), .o1(new_n131));
  inv000aa1d42x5               g036(.a(\b[9] ), .o1(new_n132));
  oao003aa1n02x5               g037(.a(new_n97), .b(new_n132), .c(new_n98), .carry(new_n133));
  aoi012aa1n02x5               g038(.a(new_n133), .b(new_n126), .c(new_n131), .o1(new_n134));
  xnrb03aa1n02x5               g039(.a(new_n134), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  oaoi03aa1n02x5               g040(.a(\a[11] ), .b(\b[10] ), .c(new_n134), .o1(new_n136));
  xorb03aa1n02x5               g041(.a(new_n136), .b(\b[11] ), .c(\a[12] ), .out0(\s[12] ));
  nona23aa1n03x5               g042(.a(new_n99), .b(new_n102), .c(new_n101), .d(new_n100), .out0(new_n138));
  oaoi03aa1n02x5               g043(.a(new_n104), .b(new_n105), .c(new_n106), .o1(new_n139));
  oabi12aa1n02x5               g044(.a(new_n109), .b(new_n138), .c(new_n139), .out0(new_n140));
  nanp02aa1n02x5               g045(.a(new_n121), .b(new_n120), .o1(new_n141));
  nanp02aa1n02x5               g046(.a(\b[3] ), .b(\a[4] ), .o1(new_n142));
  nanp02aa1n02x5               g047(.a(new_n141), .b(new_n142), .o1(new_n143));
  nanb02aa1n06x5               g048(.a(new_n112), .b(new_n113), .out0(new_n144));
  oai013aa1n06x5               g049(.a(new_n122), .b(new_n144), .c(new_n118), .d(new_n143), .o1(new_n145));
  nor003aa1n03x5               g050(.a(new_n138), .b(new_n123), .c(new_n124), .o1(new_n146));
  norp02aa1n04x5               g051(.a(\b[10] ), .b(\a[11] ), .o1(new_n147));
  nanp02aa1n02x5               g052(.a(\b[10] ), .b(\a[11] ), .o1(new_n148));
  nor022aa1n08x5               g053(.a(\b[11] ), .b(\a[12] ), .o1(new_n149));
  nand42aa1n02x5               g054(.a(\b[11] ), .b(\a[12] ), .o1(new_n150));
  nona23aa1n03x5               g055(.a(new_n150), .b(new_n148), .c(new_n147), .d(new_n149), .out0(new_n151));
  nor003aa1n02x5               g056(.a(new_n151), .b(new_n130), .c(new_n127), .o1(new_n152));
  aoai13aa1n06x5               g057(.a(new_n152), .b(new_n140), .c(new_n145), .d(new_n146), .o1(new_n153));
  nano23aa1n02x4               g058(.a(new_n147), .b(new_n149), .c(new_n150), .d(new_n148), .out0(new_n154));
  oai012aa1n02x5               g059(.a(new_n150), .b(new_n149), .c(new_n147), .o1(new_n155));
  aobi12aa1n02x5               g060(.a(new_n155), .b(new_n154), .c(new_n133), .out0(new_n156));
  nanp02aa1n02x5               g061(.a(new_n153), .b(new_n156), .o1(new_n157));
  xorb03aa1n02x5               g062(.a(new_n157), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  nor042aa1n06x5               g063(.a(\b[12] ), .b(\a[13] ), .o1(new_n159));
  nand42aa1n02x5               g064(.a(\b[12] ), .b(\a[13] ), .o1(new_n160));
  aoi012aa1n02x5               g065(.a(new_n159), .b(new_n157), .c(new_n160), .o1(new_n161));
  xnrb03aa1n03x5               g066(.a(new_n161), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  norp02aa1n06x5               g067(.a(\b[13] ), .b(\a[14] ), .o1(new_n163));
  nand22aa1n03x5               g068(.a(\b[13] ), .b(\a[14] ), .o1(new_n164));
  nona23aa1n03x5               g069(.a(new_n164), .b(new_n160), .c(new_n159), .d(new_n163), .out0(new_n165));
  aoi012aa1n02x7               g070(.a(new_n163), .b(new_n159), .c(new_n164), .o1(new_n166));
  aoai13aa1n04x5               g071(.a(new_n166), .b(new_n165), .c(new_n153), .d(new_n156), .o1(new_n167));
  xorb03aa1n02x5               g072(.a(new_n167), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  norp02aa1n02x5               g073(.a(\b[14] ), .b(\a[15] ), .o1(new_n169));
  xnrc02aa1n12x5               g074(.a(\b[14] ), .b(\a[15] ), .out0(new_n170));
  inv000aa1d42x5               g075(.a(new_n170), .o1(new_n171));
  nor002aa1n03x5               g076(.a(\b[15] ), .b(\a[16] ), .o1(new_n172));
  nand22aa1n12x5               g077(.a(\b[15] ), .b(\a[16] ), .o1(new_n173));
  norb02aa1n09x5               g078(.a(new_n173), .b(new_n172), .out0(new_n174));
  aoi112aa1n02x5               g079(.a(new_n174), .b(new_n169), .c(new_n167), .d(new_n171), .o1(new_n175));
  aoai13aa1n03x5               g080(.a(new_n174), .b(new_n169), .c(new_n167), .d(new_n171), .o1(new_n176));
  norb02aa1n02x5               g081(.a(new_n176), .b(new_n175), .out0(\s[16] ));
  nand02aa1n02x5               g082(.a(new_n145), .b(new_n146), .o1(new_n178));
  inv040aa1n02x5               g083(.a(new_n174), .o1(new_n179));
  nor043aa1n03x5               g084(.a(new_n165), .b(new_n179), .c(new_n170), .o1(new_n180));
  nand02aa1n02x5               g085(.a(new_n152), .b(new_n180), .o1(new_n181));
  oaoi03aa1n02x5               g086(.a(new_n97), .b(new_n132), .c(new_n98), .o1(new_n182));
  oai012aa1n02x7               g087(.a(new_n155), .b(new_n151), .c(new_n182), .o1(new_n183));
  oai012aa1n02x5               g088(.a(new_n173), .b(new_n172), .c(new_n169), .o1(new_n184));
  oai013aa1n03x5               g089(.a(new_n184), .b(new_n179), .c(new_n170), .d(new_n166), .o1(new_n185));
  aoi012aa1n12x5               g090(.a(new_n185), .b(new_n183), .c(new_n180), .o1(new_n186));
  aoai13aa1n06x5               g091(.a(new_n186), .b(new_n181), .c(new_n178), .d(new_n110), .o1(new_n187));
  xorb03aa1n02x5               g092(.a(new_n187), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv040aa1d32x5               g093(.a(\a[18] ), .o1(new_n189));
  inv030aa1d32x5               g094(.a(\a[17] ), .o1(new_n190));
  inv000aa1d42x5               g095(.a(\b[16] ), .o1(new_n191));
  oaoi03aa1n03x5               g096(.a(new_n190), .b(new_n191), .c(new_n187), .o1(new_n192));
  xorb03aa1n02x5               g097(.a(new_n192), .b(\b[17] ), .c(new_n189), .out0(\s[18] ));
  nano23aa1n02x4               g098(.a(new_n159), .b(new_n163), .c(new_n164), .d(new_n160), .out0(new_n194));
  nand03aa1n02x5               g099(.a(new_n171), .b(new_n194), .c(new_n174), .o1(new_n195));
  nano22aa1n03x7               g100(.a(new_n195), .b(new_n131), .c(new_n154), .out0(new_n196));
  aoai13aa1n06x5               g101(.a(new_n196), .b(new_n140), .c(new_n146), .d(new_n145), .o1(new_n197));
  xroi22aa1d06x4               g102(.a(new_n190), .b(\b[16] ), .c(new_n189), .d(\b[17] ), .out0(new_n198));
  inv000aa1d42x5               g103(.a(new_n198), .o1(new_n199));
  inv000aa1d42x5               g104(.a(\b[17] ), .o1(new_n200));
  norp02aa1n02x5               g105(.a(\b[16] ), .b(\a[17] ), .o1(new_n201));
  oao003aa1n02x5               g106(.a(new_n189), .b(new_n200), .c(new_n201), .carry(new_n202));
  inv000aa1d42x5               g107(.a(new_n202), .o1(new_n203));
  aoai13aa1n03x5               g108(.a(new_n203), .b(new_n199), .c(new_n197), .d(new_n186), .o1(new_n204));
  xorb03aa1n02x5               g109(.a(new_n204), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g110(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor002aa1n20x5               g111(.a(\b[18] ), .b(\a[19] ), .o1(new_n207));
  nand02aa1d06x5               g112(.a(\b[18] ), .b(\a[19] ), .o1(new_n208));
  nor042aa1n06x5               g113(.a(\b[19] ), .b(\a[20] ), .o1(new_n209));
  nand02aa1n08x5               g114(.a(\b[19] ), .b(\a[20] ), .o1(new_n210));
  norb02aa1n12x5               g115(.a(new_n210), .b(new_n209), .out0(new_n211));
  aoi112aa1n02x5               g116(.a(new_n207), .b(new_n211), .c(new_n204), .d(new_n208), .o1(new_n212));
  inv000aa1d42x5               g117(.a(new_n207), .o1(new_n213));
  norb02aa1n09x5               g118(.a(new_n208), .b(new_n207), .out0(new_n214));
  aoai13aa1n03x5               g119(.a(new_n214), .b(new_n202), .c(new_n187), .d(new_n198), .o1(new_n215));
  aobi12aa1n03x5               g120(.a(new_n211), .b(new_n215), .c(new_n213), .out0(new_n216));
  nor002aa1n02x5               g121(.a(new_n216), .b(new_n212), .o1(\s[20] ));
  nano23aa1n09x5               g122(.a(new_n207), .b(new_n209), .c(new_n210), .d(new_n208), .out0(new_n218));
  nand02aa1d04x5               g123(.a(new_n198), .b(new_n218), .o1(new_n219));
  aoi112aa1n02x7               g124(.a(\b[18] ), .b(\a[19] ), .c(\a[20] ), .d(\b[19] ), .o1(new_n220));
  nor042aa1n02x5               g125(.a(\b[17] ), .b(\a[18] ), .o1(new_n221));
  aoi112aa1n09x5               g126(.a(\b[16] ), .b(\a[17] ), .c(\a[18] ), .d(\b[17] ), .o1(new_n222));
  oai112aa1n06x5               g127(.a(new_n214), .b(new_n211), .c(new_n222), .d(new_n221), .o1(new_n223));
  nona22aa1d18x5               g128(.a(new_n223), .b(new_n220), .c(new_n209), .out0(new_n224));
  inv000aa1d42x5               g129(.a(new_n224), .o1(new_n225));
  aoai13aa1n06x5               g130(.a(new_n225), .b(new_n219), .c(new_n197), .d(new_n186), .o1(new_n226));
  xorb03aa1n02x5               g131(.a(new_n226), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor042aa1d18x5               g132(.a(\b[20] ), .b(\a[21] ), .o1(new_n228));
  xnrc02aa1n12x5               g133(.a(\b[20] ), .b(\a[21] ), .out0(new_n229));
  inv000aa1d42x5               g134(.a(new_n229), .o1(new_n230));
  xnrc02aa1n12x5               g135(.a(\b[21] ), .b(\a[22] ), .out0(new_n231));
  inv000aa1d42x5               g136(.a(new_n231), .o1(new_n232));
  aoi112aa1n02x5               g137(.a(new_n228), .b(new_n232), .c(new_n226), .d(new_n230), .o1(new_n233));
  inv030aa1n03x5               g138(.a(new_n228), .o1(new_n234));
  inv000aa1n02x5               g139(.a(new_n219), .o1(new_n235));
  aoai13aa1n03x5               g140(.a(new_n230), .b(new_n224), .c(new_n187), .d(new_n235), .o1(new_n236));
  aoi012aa1n03x5               g141(.a(new_n231), .b(new_n236), .c(new_n234), .o1(new_n237));
  norp02aa1n03x5               g142(.a(new_n237), .b(new_n233), .o1(\s[22] ));
  nor042aa1n06x5               g143(.a(new_n231), .b(new_n229), .o1(new_n239));
  tech160nm_fioaoi03aa1n03p5x5 g144(.a(\a[22] ), .b(\b[21] ), .c(new_n234), .o1(new_n240));
  aoi012aa1d18x5               g145(.a(new_n240), .b(new_n224), .c(new_n239), .o1(new_n241));
  nand23aa1d12x5               g146(.a(new_n198), .b(new_n239), .c(new_n218), .o1(new_n242));
  aoai13aa1n04x5               g147(.a(new_n241), .b(new_n242), .c(new_n197), .d(new_n186), .o1(new_n243));
  xorb03aa1n02x5               g148(.a(new_n243), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  nor002aa1n16x5               g149(.a(\b[22] ), .b(\a[23] ), .o1(new_n245));
  nand42aa1n06x5               g150(.a(\b[22] ), .b(\a[23] ), .o1(new_n246));
  norb02aa1n03x4               g151(.a(new_n246), .b(new_n245), .out0(new_n247));
  norp02aa1n04x5               g152(.a(\b[23] ), .b(\a[24] ), .o1(new_n248));
  nanp02aa1n04x5               g153(.a(\b[23] ), .b(\a[24] ), .o1(new_n249));
  norb02aa1n06x4               g154(.a(new_n249), .b(new_n248), .out0(new_n250));
  aoi112aa1n03x4               g155(.a(new_n245), .b(new_n250), .c(new_n243), .d(new_n247), .o1(new_n251));
  inv000aa1d42x5               g156(.a(new_n245), .o1(new_n252));
  inv000aa1d42x5               g157(.a(new_n241), .o1(new_n253));
  inv030aa1n02x5               g158(.a(new_n242), .o1(new_n254));
  aoai13aa1n03x5               g159(.a(new_n247), .b(new_n253), .c(new_n187), .d(new_n254), .o1(new_n255));
  aobi12aa1n03x5               g160(.a(new_n250), .b(new_n255), .c(new_n252), .out0(new_n256));
  nor002aa1n02x5               g161(.a(new_n256), .b(new_n251), .o1(\s[24] ));
  nona23aa1d24x5               g162(.a(new_n249), .b(new_n246), .c(new_n245), .d(new_n248), .out0(new_n258));
  inv000aa1d42x5               g163(.a(new_n258), .o1(new_n259));
  nano22aa1n03x7               g164(.a(new_n219), .b(new_n239), .c(new_n259), .out0(new_n260));
  inv020aa1n02x5               g165(.a(new_n260), .o1(new_n261));
  aoi112aa1n02x5               g166(.a(\b[22] ), .b(\a[23] ), .c(\a[24] ), .d(\b[23] ), .o1(new_n262));
  aoi113aa1n03x7               g167(.a(new_n262), .b(new_n248), .c(new_n240), .d(new_n250), .e(new_n247), .o1(new_n263));
  nona32aa1n09x5               g168(.a(new_n224), .b(new_n258), .c(new_n231), .d(new_n229), .out0(new_n264));
  nand22aa1n03x5               g169(.a(new_n264), .b(new_n263), .o1(new_n265));
  inv000aa1n02x5               g170(.a(new_n265), .o1(new_n266));
  aoai13aa1n04x5               g171(.a(new_n266), .b(new_n261), .c(new_n197), .d(new_n186), .o1(new_n267));
  xorb03aa1n02x5               g172(.a(new_n267), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g173(.a(\b[24] ), .b(\a[25] ), .o1(new_n269));
  tech160nm_fixorc02aa1n02p5x5 g174(.a(\a[25] ), .b(\b[24] ), .out0(new_n270));
  xorc02aa1n02x5               g175(.a(\a[26] ), .b(\b[25] ), .out0(new_n271));
  aoi112aa1n02x5               g176(.a(new_n269), .b(new_n271), .c(new_n267), .d(new_n270), .o1(new_n272));
  inv000aa1d42x5               g177(.a(new_n269), .o1(new_n273));
  aoai13aa1n03x5               g178(.a(new_n270), .b(new_n265), .c(new_n187), .d(new_n260), .o1(new_n274));
  aobi12aa1n03x5               g179(.a(new_n271), .b(new_n274), .c(new_n273), .out0(new_n275));
  nor002aa1n02x5               g180(.a(new_n275), .b(new_n272), .o1(\s[26] ));
  oabi12aa1n03x5               g181(.a(new_n185), .b(new_n156), .c(new_n195), .out0(new_n277));
  and002aa1n06x5               g182(.a(new_n271), .b(new_n270), .o(new_n278));
  nano22aa1d15x5               g183(.a(new_n242), .b(new_n278), .c(new_n259), .out0(new_n279));
  aoai13aa1n06x5               g184(.a(new_n279), .b(new_n277), .c(new_n126), .d(new_n196), .o1(new_n280));
  nanp02aa1n02x5               g185(.a(\b[25] ), .b(\a[26] ), .o1(new_n281));
  oai022aa1n02x5               g186(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n282));
  aoi022aa1n09x5               g187(.a(new_n265), .b(new_n278), .c(new_n281), .d(new_n282), .o1(new_n283));
  xorc02aa1n12x5               g188(.a(\a[27] ), .b(\b[26] ), .out0(new_n284));
  xnbna2aa1n03x5               g189(.a(new_n284), .b(new_n283), .c(new_n280), .out0(\s[27] ));
  norp02aa1n02x5               g190(.a(\b[26] ), .b(\a[27] ), .o1(new_n286));
  inv040aa1n03x5               g191(.a(new_n286), .o1(new_n287));
  inv000aa1d42x5               g192(.a(new_n284), .o1(new_n288));
  aoi012aa1n02x7               g193(.a(new_n288), .b(new_n283), .c(new_n280), .o1(new_n289));
  xnrc02aa1n02x5               g194(.a(\b[27] ), .b(\a[28] ), .out0(new_n290));
  nano22aa1n02x4               g195(.a(new_n289), .b(new_n287), .c(new_n290), .out0(new_n291));
  inv000aa1d42x5               g196(.a(new_n278), .o1(new_n292));
  nanp02aa1n02x5               g197(.a(new_n282), .b(new_n281), .o1(new_n293));
  aoai13aa1n06x5               g198(.a(new_n293), .b(new_n292), .c(new_n264), .d(new_n263), .o1(new_n294));
  aoai13aa1n03x5               g199(.a(new_n284), .b(new_n294), .c(new_n187), .d(new_n279), .o1(new_n295));
  tech160nm_fiaoi012aa1n02p5x5 g200(.a(new_n290), .b(new_n295), .c(new_n287), .o1(new_n296));
  norp02aa1n03x5               g201(.a(new_n296), .b(new_n291), .o1(\s[28] ));
  norb02aa1n02x7               g202(.a(new_n284), .b(new_n290), .out0(new_n298));
  aoai13aa1n03x5               g203(.a(new_n298), .b(new_n294), .c(new_n187), .d(new_n279), .o1(new_n299));
  oao003aa1n02x5               g204(.a(\a[28] ), .b(\b[27] ), .c(new_n287), .carry(new_n300));
  xnrc02aa1n02x5               g205(.a(\b[28] ), .b(\a[29] ), .out0(new_n301));
  tech160nm_fiaoi012aa1n02p5x5 g206(.a(new_n301), .b(new_n299), .c(new_n300), .o1(new_n302));
  inv000aa1d42x5               g207(.a(new_n298), .o1(new_n303));
  aoi012aa1n02x7               g208(.a(new_n303), .b(new_n283), .c(new_n280), .o1(new_n304));
  nano22aa1n03x5               g209(.a(new_n304), .b(new_n300), .c(new_n301), .out0(new_n305));
  norp02aa1n03x5               g210(.a(new_n302), .b(new_n305), .o1(\s[29] ));
  xorb03aa1n02x5               g211(.a(new_n116), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n03x5               g212(.a(new_n284), .b(new_n301), .c(new_n290), .out0(new_n308));
  aoai13aa1n03x5               g213(.a(new_n308), .b(new_n294), .c(new_n187), .d(new_n279), .o1(new_n309));
  oao003aa1n02x5               g214(.a(\a[29] ), .b(\b[28] ), .c(new_n300), .carry(new_n310));
  xnrc02aa1n02x5               g215(.a(\b[29] ), .b(\a[30] ), .out0(new_n311));
  tech160nm_fiaoi012aa1n02p5x5 g216(.a(new_n311), .b(new_n309), .c(new_n310), .o1(new_n312));
  inv000aa1d42x5               g217(.a(new_n308), .o1(new_n313));
  aoi012aa1n02x7               g218(.a(new_n313), .b(new_n283), .c(new_n280), .o1(new_n314));
  nano22aa1n03x5               g219(.a(new_n314), .b(new_n310), .c(new_n311), .out0(new_n315));
  norp02aa1n03x5               g220(.a(new_n312), .b(new_n315), .o1(\s[30] ));
  norb02aa1n03x5               g221(.a(new_n308), .b(new_n311), .out0(new_n317));
  inv000aa1n02x5               g222(.a(new_n317), .o1(new_n318));
  aoi012aa1n02x7               g223(.a(new_n318), .b(new_n283), .c(new_n280), .o1(new_n319));
  oao003aa1n02x5               g224(.a(\a[30] ), .b(\b[29] ), .c(new_n310), .carry(new_n320));
  xnrc02aa1n02x5               g225(.a(\b[30] ), .b(\a[31] ), .out0(new_n321));
  nano22aa1n02x4               g226(.a(new_n319), .b(new_n320), .c(new_n321), .out0(new_n322));
  aoai13aa1n03x5               g227(.a(new_n317), .b(new_n294), .c(new_n187), .d(new_n279), .o1(new_n323));
  tech160nm_fiaoi012aa1n02p5x5 g228(.a(new_n321), .b(new_n323), .c(new_n320), .o1(new_n324));
  norp02aa1n03x5               g229(.a(new_n324), .b(new_n322), .o1(\s[31] ));
  xnrb03aa1n02x5               g230(.a(new_n118), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g231(.a(\a[3] ), .b(\b[2] ), .c(new_n118), .o1(new_n327));
  xorb03aa1n02x5               g232(.a(new_n327), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g233(.a(new_n145), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  aoib12aa1n02x5               g234(.a(new_n106), .b(new_n145), .c(new_n124), .out0(new_n330));
  xorb03aa1n02x5               g235(.a(new_n330), .b(\b[5] ), .c(new_n104), .out0(\s[6] ));
  inv000aa1d42x5               g236(.a(new_n102), .o1(new_n332));
  and002aa1n02x5               g237(.a(\b[5] ), .b(\a[6] ), .o(new_n333));
  nanb02aa1n02x5               g238(.a(new_n123), .b(new_n330), .out0(new_n334));
  nona32aa1n02x4               g239(.a(new_n334), .b(new_n333), .c(new_n332), .d(new_n101), .out0(new_n335));
  aboi22aa1n03x5               g240(.a(new_n333), .b(new_n334), .c(new_n108), .d(new_n102), .out0(new_n336));
  norb02aa1n02x5               g241(.a(new_n335), .b(new_n336), .out0(\s[7] ));
  norb02aa1n02x5               g242(.a(new_n99), .b(new_n100), .out0(new_n338));
  xnbna2aa1n03x5               g243(.a(new_n338), .b(new_n335), .c(new_n108), .out0(\s[8] ));
  xorb03aa1n02x5               g244(.a(new_n126), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


