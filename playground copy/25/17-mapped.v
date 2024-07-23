// Benchmark "adder" written by ABC on Thu Jul 11 12:44:16 2024

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
    new_n118, new_n119, new_n120, new_n121, new_n122, new_n123, new_n125,
    new_n126, new_n127, new_n128, new_n129, new_n130, new_n131, new_n132,
    new_n133, new_n134, new_n135, new_n136, new_n138, new_n139, new_n140,
    new_n141, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n153, new_n154, new_n155, new_n156,
    new_n157, new_n159, new_n160, new_n161, new_n163, new_n164, new_n165,
    new_n166, new_n167, new_n168, new_n169, new_n170, new_n172, new_n173,
    new_n174, new_n175, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n182, new_n183, new_n184, new_n185, new_n186, new_n187, new_n188,
    new_n189, new_n190, new_n191, new_n192, new_n193, new_n194, new_n195,
    new_n196, new_n197, new_n198, new_n199, new_n201, new_n202, new_n203,
    new_n204, new_n205, new_n208, new_n209, new_n210, new_n211, new_n212,
    new_n213, new_n214, new_n215, new_n216, new_n217, new_n218, new_n219,
    new_n221, new_n222, new_n223, new_n224, new_n225, new_n226, new_n228,
    new_n229, new_n230, new_n231, new_n232, new_n233, new_n234, new_n235,
    new_n236, new_n238, new_n239, new_n240, new_n241, new_n242, new_n243,
    new_n244, new_n246, new_n247, new_n248, new_n249, new_n250, new_n251,
    new_n252, new_n253, new_n255, new_n256, new_n257, new_n258, new_n259,
    new_n260, new_n261, new_n262, new_n263, new_n264, new_n266, new_n267,
    new_n268, new_n269, new_n270, new_n271, new_n272, new_n273, new_n275,
    new_n276, new_n277, new_n278, new_n279, new_n280, new_n281, new_n282,
    new_n284, new_n285, new_n286, new_n287, new_n288, new_n289, new_n290,
    new_n291, new_n292, new_n293, new_n294, new_n295, new_n296, new_n298,
    new_n299, new_n300, new_n301, new_n302, new_n303, new_n304, new_n305,
    new_n308, new_n309, new_n310, new_n311, new_n312, new_n313, new_n314,
    new_n315, new_n317, new_n318, new_n319, new_n320, new_n321, new_n322,
    new_n323, new_n324, new_n327, new_n330, new_n332, new_n334;
  160nm_ficinv00aa1n08x5       g000(.clk(\a[0] ), .clkout(\s[0] ));
  160nm_ficinv00aa1n08x5       g001(.clk(\a[2] ), .clkout(new_n97));
  160nm_ficinv00aa1n08x5       g002(.clk(\b[1] ), .clkout(new_n98));
  nanp02aa1n02x5               g003(.a(\b[0] ), .b(\a[1] ), .o1(new_n99));
  oaoi03aa1n02x5               g004(.a(new_n97), .b(new_n98), .c(new_n99), .o1(new_n100));
  norp02aa1n02x5               g005(.a(\b[3] ), .b(\a[4] ), .o1(new_n101));
  nanp02aa1n02x5               g006(.a(\b[3] ), .b(\a[4] ), .o1(new_n102));
  norp02aa1n02x5               g007(.a(\b[2] ), .b(\a[3] ), .o1(new_n103));
  nanp02aa1n02x5               g008(.a(\b[2] ), .b(\a[3] ), .o1(new_n104));
  nona23aa1n02x4               g009(.a(new_n104), .b(new_n102), .c(new_n101), .d(new_n103), .out0(new_n105));
  160nm_fiao0012aa1n02p5x5     g010(.a(new_n101), .b(new_n103), .c(new_n102), .o(new_n106));
  oabi12aa1n02x5               g011(.a(new_n106), .b(new_n105), .c(new_n100), .out0(new_n107));
  xnrc02aa1n02x5               g012(.a(\b[5] ), .b(\a[6] ), .out0(new_n108));
  norp02aa1n02x5               g013(.a(\b[7] ), .b(\a[8] ), .o1(new_n109));
  nanp02aa1n02x5               g014(.a(\b[7] ), .b(\a[8] ), .o1(new_n110));
  norp02aa1n02x5               g015(.a(\b[6] ), .b(\a[7] ), .o1(new_n111));
  nanp02aa1n02x5               g016(.a(\b[6] ), .b(\a[7] ), .o1(new_n112));
  nona23aa1n02x4               g017(.a(new_n112), .b(new_n110), .c(new_n109), .d(new_n111), .out0(new_n113));
  xnrc02aa1n02x5               g018(.a(\b[4] ), .b(\a[5] ), .out0(new_n114));
  norp03aa1n02x5               g019(.a(new_n113), .b(new_n114), .c(new_n108), .o1(new_n115));
  oai012aa1n02x5               g020(.a(new_n110), .b(new_n111), .c(new_n109), .o1(new_n116));
  160nm_ficinv00aa1n08x5       g021(.clk(\a[5] ), .clkout(new_n117));
  160nm_ficinv00aa1n08x5       g022(.clk(\b[4] ), .clkout(new_n118));
  nanp02aa1n02x5               g023(.a(new_n118), .b(new_n117), .o1(new_n119));
  oaoi03aa1n02x5               g024(.a(\a[6] ), .b(\b[5] ), .c(new_n119), .o1(new_n120));
  oaib12aa1n02x5               g025(.a(new_n116), .b(new_n113), .c(new_n120), .out0(new_n121));
  aoi012aa1n02x5               g026(.a(new_n121), .b(new_n107), .c(new_n115), .o1(new_n122));
  oaoi03aa1n02x5               g027(.a(\a[9] ), .b(\b[8] ), .c(new_n122), .o1(new_n123));
  xorb03aa1n02x5               g028(.a(new_n123), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  norp02aa1n02x5               g029(.a(\b[10] ), .b(\a[11] ), .o1(new_n125));
  nanp02aa1n02x5               g030(.a(\b[10] ), .b(\a[11] ), .o1(new_n126));
  norb02aa1n02x5               g031(.a(new_n126), .b(new_n125), .out0(new_n127));
  160nm_ficinv00aa1n08x5       g032(.clk(new_n127), .clkout(new_n128));
  norp02aa1n02x5               g033(.a(\b[8] ), .b(\a[9] ), .o1(new_n129));
  norp02aa1n02x5               g034(.a(\b[9] ), .b(\a[10] ), .o1(new_n130));
  nanp02aa1n02x5               g035(.a(\b[9] ), .b(\a[10] ), .o1(new_n131));
  oai012aa1n02x5               g036(.a(new_n131), .b(new_n130), .c(new_n129), .o1(new_n132));
  nanp02aa1n02x5               g037(.a(\b[8] ), .b(\a[9] ), .o1(new_n133));
  nona23aa1n02x4               g038(.a(new_n131), .b(new_n133), .c(new_n129), .d(new_n130), .out0(new_n134));
  oaoi13aa1n02x5               g039(.a(new_n128), .b(new_n132), .c(new_n122), .d(new_n134), .o1(new_n135));
  oai112aa1n02x5               g040(.a(new_n132), .b(new_n128), .c(new_n122), .d(new_n134), .o1(new_n136));
  norb02aa1n02x5               g041(.a(new_n136), .b(new_n135), .out0(\s[11] ));
  norp02aa1n02x5               g042(.a(new_n135), .b(new_n125), .o1(new_n138));
  norp02aa1n02x5               g043(.a(\b[11] ), .b(\a[12] ), .o1(new_n139));
  160nm_ficinv00aa1n08x5       g044(.clk(new_n139), .clkout(new_n140));
  nanp02aa1n02x5               g045(.a(\b[11] ), .b(\a[12] ), .o1(new_n141));
  xnbna2aa1n03x5               g046(.a(new_n138), .b(new_n141), .c(new_n140), .out0(\s[12] ));
  nona23aa1n02x4               g047(.a(new_n141), .b(new_n126), .c(new_n125), .d(new_n139), .out0(new_n143));
  norp02aa1n02x5               g048(.a(new_n143), .b(new_n134), .o1(new_n144));
  aoai13aa1n02x5               g049(.a(new_n144), .b(new_n121), .c(new_n107), .d(new_n115), .o1(new_n145));
  nanp02aa1n02x5               g050(.a(new_n125), .b(new_n141), .o1(new_n146));
  oai112aa1n02x5               g051(.a(new_n146), .b(new_n140), .c(new_n143), .d(new_n132), .o1(new_n147));
  160nm_ficinv00aa1n08x5       g052(.clk(new_n147), .clkout(new_n148));
  norp02aa1n02x5               g053(.a(\b[12] ), .b(\a[13] ), .o1(new_n149));
  nanp02aa1n02x5               g054(.a(\b[12] ), .b(\a[13] ), .o1(new_n150));
  norb02aa1n02x5               g055(.a(new_n150), .b(new_n149), .out0(new_n151));
  xnbna2aa1n03x5               g056(.a(new_n151), .b(new_n145), .c(new_n148), .out0(\s[13] ));
  160nm_ficinv00aa1n08x5       g057(.clk(new_n149), .clkout(new_n153));
  aob012aa1n02x5               g058(.a(new_n151), .b(new_n145), .c(new_n148), .out0(new_n154));
  norp02aa1n02x5               g059(.a(\b[13] ), .b(\a[14] ), .o1(new_n155));
  nanp02aa1n02x5               g060(.a(\b[13] ), .b(\a[14] ), .o1(new_n156));
  norb02aa1n02x5               g061(.a(new_n156), .b(new_n155), .out0(new_n157));
  xnbna2aa1n03x5               g062(.a(new_n157), .b(new_n154), .c(new_n153), .out0(\s[14] ));
  nona23aa1n02x4               g063(.a(new_n156), .b(new_n150), .c(new_n149), .d(new_n155), .out0(new_n159));
  oai012aa1n02x5               g064(.a(new_n156), .b(new_n155), .c(new_n149), .o1(new_n160));
  aoai13aa1n02x5               g065(.a(new_n160), .b(new_n159), .c(new_n145), .d(new_n148), .o1(new_n161));
  xorb03aa1n02x5               g066(.a(new_n161), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  norp02aa1n02x5               g067(.a(\b[14] ), .b(\a[15] ), .o1(new_n163));
  nanp02aa1n02x5               g068(.a(\b[14] ), .b(\a[15] ), .o1(new_n164));
  norb02aa1n02x5               g069(.a(new_n164), .b(new_n163), .out0(new_n165));
  norp02aa1n02x5               g070(.a(\b[15] ), .b(\a[16] ), .o1(new_n166));
  nanp02aa1n02x5               g071(.a(\b[15] ), .b(\a[16] ), .o1(new_n167));
  norb02aa1n02x5               g072(.a(new_n167), .b(new_n166), .out0(new_n168));
  aoi112aa1n02x5               g073(.a(new_n168), .b(new_n163), .c(new_n161), .d(new_n165), .o1(new_n169));
  aoai13aa1n02x5               g074(.a(new_n168), .b(new_n163), .c(new_n161), .d(new_n164), .o1(new_n170));
  norb02aa1n02x5               g075(.a(new_n170), .b(new_n169), .out0(\s[16] ));
  nano23aa1n02x4               g076(.a(new_n129), .b(new_n130), .c(new_n131), .d(new_n133), .out0(new_n172));
  nano23aa1n02x4               g077(.a(new_n163), .b(new_n166), .c(new_n167), .d(new_n164), .out0(new_n173));
  nano23aa1n02x4               g078(.a(new_n159), .b(new_n143), .c(new_n173), .d(new_n172), .out0(new_n174));
  aoai13aa1n02x5               g079(.a(new_n174), .b(new_n121), .c(new_n107), .d(new_n115), .o1(new_n175));
  nano22aa1n02x4               g080(.a(new_n159), .b(new_n165), .c(new_n168), .out0(new_n176));
  aoi112aa1n02x5               g081(.a(\b[14] ), .b(\a[15] ), .c(\a[16] ), .d(\b[15] ), .o1(new_n177));
  obai22aa1n02x7               g082(.a(new_n173), .b(new_n160), .c(\a[16] ), .d(\b[15] ), .out0(new_n178));
  aoi112aa1n02x5               g083(.a(new_n178), .b(new_n177), .c(new_n147), .d(new_n176), .o1(new_n179));
  xorc02aa1n02x5               g084(.a(\a[17] ), .b(\b[16] ), .out0(new_n180));
  xnbna2aa1n03x5               g085(.a(new_n180), .b(new_n179), .c(new_n175), .out0(\s[17] ));
  160nm_ficinv00aa1n08x5       g086(.clk(\a[17] ), .clkout(new_n182));
  160nm_ficinv00aa1n08x5       g087(.clk(\b[16] ), .clkout(new_n183));
  nanp02aa1n02x5               g088(.a(new_n183), .b(new_n182), .o1(new_n184));
  oao003aa1n02x5               g089(.a(new_n97), .b(new_n98), .c(new_n99), .carry(new_n185));
  nano23aa1n02x4               g090(.a(new_n101), .b(new_n103), .c(new_n104), .d(new_n102), .out0(new_n186));
  aoi012aa1n02x5               g091(.a(new_n106), .b(new_n186), .c(new_n185), .o1(new_n187));
  nano23aa1n02x4               g092(.a(new_n109), .b(new_n111), .c(new_n112), .d(new_n110), .out0(new_n188));
  nona22aa1n02x4               g093(.a(new_n188), .b(new_n114), .c(new_n108), .out0(new_n189));
  aobi12aa1n02x5               g094(.a(new_n116), .b(new_n188), .c(new_n120), .out0(new_n190));
  nanp02aa1n02x5               g095(.a(new_n176), .b(new_n144), .o1(new_n191));
  oaoi13aa1n02x5               g096(.a(new_n191), .b(new_n190), .c(new_n187), .d(new_n189), .o1(new_n192));
  nanp02aa1n02x5               g097(.a(new_n147), .b(new_n176), .o1(new_n193));
  nano22aa1n02x4               g098(.a(new_n160), .b(new_n165), .c(new_n168), .out0(new_n194));
  nona32aa1n02x4               g099(.a(new_n193), .b(new_n194), .c(new_n177), .d(new_n166), .out0(new_n195));
  oai012aa1n02x5               g100(.a(new_n180), .b(new_n195), .c(new_n192), .o1(new_n196));
  norp02aa1n02x5               g101(.a(\b[17] ), .b(\a[18] ), .o1(new_n197));
  nanp02aa1n02x5               g102(.a(\b[17] ), .b(\a[18] ), .o1(new_n198));
  nanb02aa1n02x5               g103(.a(new_n197), .b(new_n198), .out0(new_n199));
  xobna2aa1n03x5               g104(.a(new_n199), .b(new_n196), .c(new_n184), .out0(\s[18] ));
  nanp02aa1n02x5               g105(.a(\b[16] ), .b(\a[17] ), .o1(new_n201));
  nano22aa1n02x4               g106(.a(new_n199), .b(new_n184), .c(new_n201), .out0(new_n202));
  160nm_ficinv00aa1n08x5       g107(.clk(new_n202), .clkout(new_n203));
  aoai13aa1n02x5               g108(.a(new_n198), .b(new_n197), .c(new_n182), .d(new_n183), .o1(new_n204));
  aoai13aa1n02x5               g109(.a(new_n204), .b(new_n203), .c(new_n179), .d(new_n175), .o1(new_n205));
  xorb03aa1n02x5               g110(.a(new_n205), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g111(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  norp02aa1n02x5               g112(.a(\b[18] ), .b(\a[19] ), .o1(new_n208));
  nanp02aa1n02x5               g113(.a(\b[18] ), .b(\a[19] ), .o1(new_n209));
  nanb02aa1n02x5               g114(.a(new_n208), .b(new_n209), .out0(new_n210));
  160nm_ficinv00aa1n08x5       g115(.clk(new_n210), .clkout(new_n211));
  norp02aa1n02x5               g116(.a(\b[19] ), .b(\a[20] ), .o1(new_n212));
  nanp02aa1n02x5               g117(.a(\b[19] ), .b(\a[20] ), .o1(new_n213));
  nanb02aa1n02x5               g118(.a(new_n212), .b(new_n213), .out0(new_n214));
  160nm_ficinv00aa1n08x5       g119(.clk(new_n214), .clkout(new_n215));
  aoi112aa1n02x5               g120(.a(new_n208), .b(new_n215), .c(new_n205), .d(new_n211), .o1(new_n216));
  160nm_ficinv00aa1n08x5       g121(.clk(new_n208), .clkout(new_n217));
  nanp02aa1n02x5               g122(.a(new_n205), .b(new_n211), .o1(new_n218));
  aoi012aa1n02x5               g123(.a(new_n214), .b(new_n218), .c(new_n217), .o1(new_n219));
  norp02aa1n02x5               g124(.a(new_n219), .b(new_n216), .o1(\s[20] ));
  nona22aa1n02x4               g125(.a(new_n202), .b(new_n210), .c(new_n214), .out0(new_n221));
  160nm_ficinv00aa1n08x5       g126(.clk(new_n212), .clkout(new_n222));
  nanp02aa1n02x5               g127(.a(new_n208), .b(new_n213), .o1(new_n223));
  norp03aa1n02x5               g128(.a(new_n204), .b(new_n210), .c(new_n214), .o1(new_n224));
  nano22aa1n02x4               g129(.a(new_n224), .b(new_n222), .c(new_n223), .out0(new_n225));
  aoai13aa1n02x5               g130(.a(new_n225), .b(new_n221), .c(new_n179), .d(new_n175), .o1(new_n226));
  xorb03aa1n02x5               g131(.a(new_n226), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  norp02aa1n02x5               g132(.a(\b[20] ), .b(\a[21] ), .o1(new_n228));
  xnrc02aa1n02x5               g133(.a(\b[20] ), .b(\a[21] ), .out0(new_n229));
  160nm_ficinv00aa1n08x5       g134(.clk(new_n229), .clkout(new_n230));
  xnrc02aa1n02x5               g135(.a(\b[21] ), .b(\a[22] ), .out0(new_n231));
  160nm_ficinv00aa1n08x5       g136(.clk(new_n231), .clkout(new_n232));
  aoi112aa1n02x5               g137(.a(new_n228), .b(new_n232), .c(new_n226), .d(new_n230), .o1(new_n233));
  160nm_ficinv00aa1n08x5       g138(.clk(new_n228), .clkout(new_n234));
  nanp02aa1n02x5               g139(.a(new_n226), .b(new_n230), .o1(new_n235));
  aoi012aa1n02x5               g140(.a(new_n231), .b(new_n235), .c(new_n234), .o1(new_n236));
  norp02aa1n02x5               g141(.a(new_n236), .b(new_n233), .o1(\s[22] ));
  nona23aa1n02x4               g142(.a(new_n213), .b(new_n209), .c(new_n208), .d(new_n212), .out0(new_n238));
  norp02aa1n02x5               g143(.a(new_n231), .b(new_n229), .o1(new_n239));
  nanb03aa1n02x5               g144(.a(new_n238), .b(new_n239), .c(new_n202), .out0(new_n240));
  oai112aa1n02x5               g145(.a(new_n223), .b(new_n222), .c(new_n238), .d(new_n204), .o1(new_n241));
  oaoi03aa1n02x5               g146(.a(\a[22] ), .b(\b[21] ), .c(new_n234), .o1(new_n242));
  aoi012aa1n02x5               g147(.a(new_n242), .b(new_n241), .c(new_n239), .o1(new_n243));
  aoai13aa1n02x5               g148(.a(new_n243), .b(new_n240), .c(new_n179), .d(new_n175), .o1(new_n244));
  xorb03aa1n02x5               g149(.a(new_n244), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g150(.a(\b[22] ), .b(\a[23] ), .o1(new_n246));
  xorc02aa1n02x5               g151(.a(\a[23] ), .b(\b[22] ), .out0(new_n247));
  xorc02aa1n02x5               g152(.a(\a[24] ), .b(\b[23] ), .out0(new_n248));
  aoi112aa1n02x5               g153(.a(new_n246), .b(new_n248), .c(new_n244), .d(new_n247), .o1(new_n249));
  160nm_ficinv00aa1n08x5       g154(.clk(new_n246), .clkout(new_n250));
  nanp02aa1n02x5               g155(.a(new_n244), .b(new_n247), .o1(new_n251));
  160nm_ficinv00aa1n08x5       g156(.clk(new_n248), .clkout(new_n252));
  aoi012aa1n02x5               g157(.a(new_n252), .b(new_n251), .c(new_n250), .o1(new_n253));
  norp02aa1n02x5               g158(.a(new_n253), .b(new_n249), .o1(\s[24] ));
  nanp02aa1n02x5               g159(.a(new_n248), .b(new_n247), .o1(new_n255));
  nona23aa1n02x4               g160(.a(new_n239), .b(new_n202), .c(new_n255), .d(new_n238), .out0(new_n256));
  xnrc02aa1n02x5               g161(.a(\b[22] ), .b(\a[23] ), .out0(new_n257));
  norb02aa1n02x5               g162(.a(new_n248), .b(new_n257), .out0(new_n258));
  norp02aa1n02x5               g163(.a(\b[23] ), .b(\a[24] ), .o1(new_n259));
  aoi112aa1n02x5               g164(.a(\b[22] ), .b(\a[23] ), .c(\a[24] ), .d(\b[23] ), .o1(new_n260));
  nanp03aa1n02x5               g165(.a(new_n242), .b(new_n247), .c(new_n248), .o1(new_n261));
  nona22aa1n02x4               g166(.a(new_n261), .b(new_n260), .c(new_n259), .out0(new_n262));
  aoi013aa1n02x4               g167(.a(new_n262), .b(new_n241), .c(new_n239), .d(new_n258), .o1(new_n263));
  aoai13aa1n02x5               g168(.a(new_n263), .b(new_n256), .c(new_n179), .d(new_n175), .o1(new_n264));
  xorb03aa1n02x5               g169(.a(new_n264), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g170(.a(\b[24] ), .b(\a[25] ), .o1(new_n266));
  xorc02aa1n02x5               g171(.a(\a[25] ), .b(\b[24] ), .out0(new_n267));
  xorc02aa1n02x5               g172(.a(\a[26] ), .b(\b[25] ), .out0(new_n268));
  aoi112aa1n02x5               g173(.a(new_n266), .b(new_n268), .c(new_n264), .d(new_n267), .o1(new_n269));
  160nm_ficinv00aa1n08x5       g174(.clk(new_n266), .clkout(new_n270));
  nanp02aa1n02x5               g175(.a(new_n264), .b(new_n267), .o1(new_n271));
  160nm_ficinv00aa1n08x5       g176(.clk(new_n268), .clkout(new_n272));
  aoi012aa1n02x5               g177(.a(new_n272), .b(new_n271), .c(new_n270), .o1(new_n273));
  norp02aa1n02x5               g178(.a(new_n273), .b(new_n269), .o1(\s[26] ));
  and002aa1n02x5               g179(.a(new_n268), .b(new_n267), .o(new_n275));
  nano22aa1n02x4               g180(.a(new_n240), .b(new_n275), .c(new_n258), .out0(new_n276));
  oai012aa1n02x5               g181(.a(new_n276), .b(new_n195), .c(new_n192), .o1(new_n277));
  nano22aa1n02x4               g182(.a(new_n225), .b(new_n239), .c(new_n258), .out0(new_n278));
  oao003aa1n02x5               g183(.a(\a[26] ), .b(\b[25] ), .c(new_n270), .carry(new_n279));
  160nm_ficinv00aa1n08x5       g184(.clk(new_n279), .clkout(new_n280));
  oaoi13aa1n02x5               g185(.a(new_n280), .b(new_n275), .c(new_n278), .d(new_n262), .o1(new_n281));
  xorc02aa1n02x5               g186(.a(\a[27] ), .b(\b[26] ), .out0(new_n282));
  xnbna2aa1n03x5               g187(.a(new_n282), .b(new_n277), .c(new_n281), .out0(\s[27] ));
  norp02aa1n02x5               g188(.a(\b[26] ), .b(\a[27] ), .o1(new_n284));
  160nm_ficinv00aa1n08x5       g189(.clk(new_n284), .clkout(new_n285));
  160nm_ficinv00aa1n08x5       g190(.clk(new_n282), .clkout(new_n286));
  aoi012aa1n02x5               g191(.a(new_n286), .b(new_n277), .c(new_n281), .o1(new_n287));
  xnrc02aa1n02x5               g192(.a(\b[27] ), .b(\a[28] ), .out0(new_n288));
  nano22aa1n02x4               g193(.a(new_n287), .b(new_n285), .c(new_n288), .out0(new_n289));
  nanp02aa1n02x5               g194(.a(new_n179), .b(new_n175), .o1(new_n290));
  nona32aa1n02x4               g195(.a(new_n241), .b(new_n255), .c(new_n231), .d(new_n229), .out0(new_n291));
  160nm_ficinv00aa1n08x5       g196(.clk(new_n262), .clkout(new_n292));
  160nm_ficinv00aa1n08x5       g197(.clk(new_n275), .clkout(new_n293));
  aoai13aa1n02x5               g198(.a(new_n279), .b(new_n293), .c(new_n291), .d(new_n292), .o1(new_n294));
  aoai13aa1n02x5               g199(.a(new_n282), .b(new_n294), .c(new_n290), .d(new_n276), .o1(new_n295));
  aoi012aa1n02x5               g200(.a(new_n288), .b(new_n295), .c(new_n285), .o1(new_n296));
  norp02aa1n02x5               g201(.a(new_n296), .b(new_n289), .o1(\s[28] ));
  norb02aa1n02x5               g202(.a(new_n282), .b(new_n288), .out0(new_n298));
  aoai13aa1n02x5               g203(.a(new_n298), .b(new_n294), .c(new_n290), .d(new_n276), .o1(new_n299));
  oao003aa1n02x5               g204(.a(\a[28] ), .b(\b[27] ), .c(new_n285), .carry(new_n300));
  xnrc02aa1n02x5               g205(.a(\b[28] ), .b(\a[29] ), .out0(new_n301));
  aoi012aa1n02x5               g206(.a(new_n301), .b(new_n299), .c(new_n300), .o1(new_n302));
  160nm_ficinv00aa1n08x5       g207(.clk(new_n298), .clkout(new_n303));
  aoi012aa1n02x5               g208(.a(new_n303), .b(new_n277), .c(new_n281), .o1(new_n304));
  nano22aa1n02x4               g209(.a(new_n304), .b(new_n300), .c(new_n301), .out0(new_n305));
  norp02aa1n02x5               g210(.a(new_n302), .b(new_n305), .o1(\s[29] ));
  xorb03aa1n02x5               g211(.a(new_n99), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g212(.a(new_n282), .b(new_n301), .c(new_n288), .out0(new_n308));
  aoai13aa1n02x5               g213(.a(new_n308), .b(new_n294), .c(new_n290), .d(new_n276), .o1(new_n309));
  oao003aa1n02x5               g214(.a(\a[29] ), .b(\b[28] ), .c(new_n300), .carry(new_n310));
  xnrc02aa1n02x5               g215(.a(\b[29] ), .b(\a[30] ), .out0(new_n311));
  aoi012aa1n02x5               g216(.a(new_n311), .b(new_n309), .c(new_n310), .o1(new_n312));
  160nm_ficinv00aa1n08x5       g217(.clk(new_n308), .clkout(new_n313));
  aoi012aa1n02x5               g218(.a(new_n313), .b(new_n277), .c(new_n281), .o1(new_n314));
  nano22aa1n02x4               g219(.a(new_n314), .b(new_n310), .c(new_n311), .out0(new_n315));
  norp02aa1n02x5               g220(.a(new_n312), .b(new_n315), .o1(\s[30] ));
  norb02aa1n02x5               g221(.a(new_n308), .b(new_n311), .out0(new_n317));
  160nm_ficinv00aa1n08x5       g222(.clk(new_n317), .clkout(new_n318));
  aoi012aa1n02x5               g223(.a(new_n318), .b(new_n277), .c(new_n281), .o1(new_n319));
  oao003aa1n02x5               g224(.a(\a[30] ), .b(\b[29] ), .c(new_n310), .carry(new_n320));
  xnrc02aa1n02x5               g225(.a(\b[30] ), .b(\a[31] ), .out0(new_n321));
  nano22aa1n02x4               g226(.a(new_n319), .b(new_n320), .c(new_n321), .out0(new_n322));
  aoai13aa1n02x5               g227(.a(new_n317), .b(new_n294), .c(new_n290), .d(new_n276), .o1(new_n323));
  aoi012aa1n02x5               g228(.a(new_n321), .b(new_n323), .c(new_n320), .o1(new_n324));
  norp02aa1n02x5               g229(.a(new_n324), .b(new_n322), .o1(\s[31] ));
  xnrb03aa1n02x5               g230(.a(new_n100), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g231(.a(\a[3] ), .b(\b[2] ), .c(new_n100), .o1(new_n327));
  xorb03aa1n02x5               g232(.a(new_n327), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g233(.a(new_n107), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oaoi03aa1n02x5               g234(.a(new_n117), .b(new_n118), .c(new_n107), .o1(new_n330));
  xnrb03aa1n02x5               g235(.a(new_n330), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  oaoi03aa1n02x5               g236(.a(\a[6] ), .b(\b[5] ), .c(new_n330), .o1(new_n332));
  xorb03aa1n02x5               g237(.a(new_n332), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoi012aa1n02x5               g238(.a(new_n111), .b(new_n332), .c(new_n112), .o1(new_n334));
  xnrb03aa1n02x5               g239(.a(new_n334), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xnrb03aa1n02x5               g240(.a(new_n122), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


