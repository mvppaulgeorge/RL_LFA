// Benchmark "adder" written by ABC on Thu Jul 11 11:35:56 2024

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
    new_n134, new_n135, new_n136, new_n137, new_n138, new_n139, new_n140,
    new_n141, new_n142, new_n143, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n162, new_n163,
    new_n164, new_n165, new_n167, new_n168, new_n169, new_n170, new_n171,
    new_n172, new_n173, new_n175, new_n176, new_n177, new_n178, new_n179,
    new_n180, new_n181, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n193, new_n194, new_n195,
    new_n196, new_n198, new_n199, new_n200, new_n201, new_n202, new_n203,
    new_n206, new_n207, new_n208, new_n209, new_n210, new_n211, new_n212,
    new_n213, new_n214, new_n215, new_n217, new_n218, new_n219, new_n220,
    new_n221, new_n222, new_n224, new_n225, new_n226, new_n227, new_n228,
    new_n229, new_n231, new_n232, new_n233, new_n234, new_n235, new_n236,
    new_n237, new_n238, new_n239, new_n241, new_n242, new_n243, new_n244,
    new_n245, new_n246, new_n247, new_n249, new_n250, new_n251, new_n252,
    new_n253, new_n254, new_n255, new_n256, new_n257, new_n258, new_n259,
    new_n261, new_n262, new_n263, new_n264, new_n265, new_n266, new_n267,
    new_n269, new_n270, new_n271, new_n272, new_n273, new_n274, new_n275,
    new_n276, new_n278, new_n279, new_n280, new_n281, new_n282, new_n283,
    new_n284, new_n285, new_n286, new_n288, new_n289, new_n290, new_n291,
    new_n292, new_n293, new_n294, new_n295, new_n298, new_n299, new_n300,
    new_n301, new_n302, new_n303, new_n305, new_n306, new_n307, new_n308,
    new_n309, new_n310, new_n311, new_n314, new_n317, new_n319, new_n321;
  160nm_ficinv00aa1n08x5       g000(.clk(\a[0] ), .clkout(\s[0] ));
  norp02aa1n02x5               g001(.a(\b[9] ), .b(\a[10] ), .o1(new_n97));
  nanp02aa1n02x5               g002(.a(\b[9] ), .b(\a[10] ), .o1(new_n98));
  norb02aa1n02x5               g003(.a(new_n98), .b(new_n97), .out0(new_n99));
  norp02aa1n02x5               g004(.a(\b[8] ), .b(\a[9] ), .o1(new_n100));
  160nm_ficinv00aa1n08x5       g005(.clk(new_n100), .clkout(new_n101));
  norp02aa1n02x5               g006(.a(\b[3] ), .b(\a[4] ), .o1(new_n102));
  nanp02aa1n02x5               g007(.a(\b[3] ), .b(\a[4] ), .o1(new_n103));
  norp02aa1n02x5               g008(.a(\b[2] ), .b(\a[3] ), .o1(new_n104));
  nanp02aa1n02x5               g009(.a(\b[2] ), .b(\a[3] ), .o1(new_n105));
  nano23aa1n02x4               g010(.a(new_n102), .b(new_n104), .c(new_n105), .d(new_n103), .out0(new_n106));
  and002aa1n02x5               g011(.a(\b[1] ), .b(\a[2] ), .o(new_n107));
  nanp02aa1n02x5               g012(.a(\b[0] ), .b(\a[1] ), .o1(new_n108));
  norp02aa1n02x5               g013(.a(\b[1] ), .b(\a[2] ), .o1(new_n109));
  oab012aa1n02x4               g014(.a(new_n107), .b(new_n109), .c(new_n108), .out0(new_n110));
  nanp02aa1n02x5               g015(.a(new_n106), .b(new_n110), .o1(new_n111));
  aoi012aa1n02x5               g016(.a(new_n102), .b(new_n104), .c(new_n103), .o1(new_n112));
  norp02aa1n02x5               g017(.a(\b[7] ), .b(\a[8] ), .o1(new_n113));
  nanp02aa1n02x5               g018(.a(\b[7] ), .b(\a[8] ), .o1(new_n114));
  norp02aa1n02x5               g019(.a(\b[6] ), .b(\a[7] ), .o1(new_n115));
  nanp02aa1n02x5               g020(.a(\b[6] ), .b(\a[7] ), .o1(new_n116));
  nano23aa1n02x4               g021(.a(new_n113), .b(new_n115), .c(new_n116), .d(new_n114), .out0(new_n117));
  norp02aa1n02x5               g022(.a(\b[5] ), .b(\a[6] ), .o1(new_n118));
  nanp02aa1n02x5               g023(.a(\b[5] ), .b(\a[6] ), .o1(new_n119));
  norp02aa1n02x5               g024(.a(\b[4] ), .b(\a[5] ), .o1(new_n120));
  nanp02aa1n02x5               g025(.a(\b[4] ), .b(\a[5] ), .o1(new_n121));
  nano23aa1n02x4               g026(.a(new_n118), .b(new_n120), .c(new_n121), .d(new_n119), .out0(new_n122));
  nanp02aa1n02x5               g027(.a(new_n122), .b(new_n117), .o1(new_n123));
  aoi012aa1n02x5               g028(.a(new_n118), .b(new_n120), .c(new_n119), .o1(new_n124));
  160nm_ficinv00aa1n08x5       g029(.clk(new_n124), .clkout(new_n125));
  160nm_fiao0012aa1n02p5x5     g030(.a(new_n113), .b(new_n115), .c(new_n114), .o(new_n126));
  aoi012aa1n02x5               g031(.a(new_n126), .b(new_n117), .c(new_n125), .o1(new_n127));
  aoai13aa1n02x5               g032(.a(new_n127), .b(new_n123), .c(new_n111), .d(new_n112), .o1(new_n128));
  aob012aa1n02x5               g033(.a(new_n128), .b(\b[8] ), .c(\a[9] ), .out0(new_n129));
  xnbna2aa1n03x5               g034(.a(new_n99), .b(new_n129), .c(new_n101), .out0(\s[10] ));
  160nm_ficinv00aa1n08x5       g035(.clk(new_n98), .clkout(new_n131));
  aoi013aa1n02x4               g036(.a(new_n131), .b(new_n129), .c(new_n101), .d(new_n99), .o1(new_n132));
  xorb03aa1n02x5               g037(.a(new_n132), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  norp02aa1n02x5               g038(.a(\b[10] ), .b(\a[11] ), .o1(new_n134));
  nanp02aa1n02x5               g039(.a(\b[10] ), .b(\a[11] ), .o1(new_n135));
  norb02aa1n02x5               g040(.a(new_n135), .b(new_n134), .out0(new_n136));
  norp02aa1n02x5               g041(.a(\b[11] ), .b(\a[12] ), .o1(new_n137));
  nanp02aa1n02x5               g042(.a(\b[11] ), .b(\a[12] ), .o1(new_n138));
  norb02aa1n02x5               g043(.a(new_n138), .b(new_n137), .out0(new_n139));
  160nm_ficinv00aa1n08x5       g044(.clk(new_n139), .clkout(new_n140));
  aoai13aa1n02x5               g045(.a(new_n140), .b(new_n134), .c(new_n132), .d(new_n136), .o1(new_n141));
  nanp02aa1n02x5               g046(.a(new_n132), .b(new_n136), .o1(new_n142));
  nona22aa1n02x4               g047(.a(new_n142), .b(new_n140), .c(new_n134), .out0(new_n143));
  nanp02aa1n02x5               g048(.a(new_n143), .b(new_n141), .o1(\s[12] ));
  aoi012aa1n02x5               g049(.a(new_n97), .b(new_n100), .c(new_n98), .o1(new_n145));
  nanb03aa1n02x5               g050(.a(new_n145), .b(new_n136), .c(new_n139), .out0(new_n146));
  aoi012aa1n02x5               g051(.a(new_n137), .b(new_n134), .c(new_n138), .o1(new_n147));
  and002aa1n02x5               g052(.a(new_n146), .b(new_n147), .o(new_n148));
  nona23aa1n02x4               g053(.a(new_n105), .b(new_n103), .c(new_n102), .d(new_n104), .out0(new_n149));
  oabi12aa1n02x5               g054(.a(new_n107), .b(new_n108), .c(new_n109), .out0(new_n150));
  oai012aa1n02x5               g055(.a(new_n112), .b(new_n149), .c(new_n150), .o1(new_n151));
  nona23aa1n02x4               g056(.a(new_n116), .b(new_n114), .c(new_n113), .d(new_n115), .out0(new_n152));
  nona23aa1n02x4               g057(.a(new_n121), .b(new_n119), .c(new_n118), .d(new_n120), .out0(new_n153));
  norp02aa1n02x5               g058(.a(new_n153), .b(new_n152), .o1(new_n154));
  oabi12aa1n02x5               g059(.a(new_n126), .b(new_n152), .c(new_n124), .out0(new_n155));
  nano23aa1n02x4               g060(.a(new_n134), .b(new_n137), .c(new_n138), .d(new_n135), .out0(new_n156));
  xorc02aa1n02x5               g061(.a(\a[9] ), .b(\b[8] ), .out0(new_n157));
  and003aa1n02x5               g062(.a(new_n156), .b(new_n99), .c(new_n157), .o(new_n158));
  aoai13aa1n02x5               g063(.a(new_n158), .b(new_n155), .c(new_n151), .d(new_n154), .o1(new_n159));
  xorc02aa1n02x5               g064(.a(\a[13] ), .b(\b[12] ), .out0(new_n160));
  xnbna2aa1n03x5               g065(.a(new_n160), .b(new_n159), .c(new_n148), .out0(\s[13] ));
  160nm_ficinv00aa1n08x5       g066(.clk(\a[14] ), .clkout(new_n162));
  nanp02aa1n02x5               g067(.a(new_n159), .b(new_n148), .o1(new_n163));
  norp02aa1n02x5               g068(.a(\b[12] ), .b(\a[13] ), .o1(new_n164));
  aoi012aa1n02x5               g069(.a(new_n164), .b(new_n163), .c(new_n160), .o1(new_n165));
  xorb03aa1n02x5               g070(.a(new_n165), .b(\b[13] ), .c(new_n162), .out0(\s[14] ));
  xorc02aa1n02x5               g071(.a(\a[14] ), .b(\b[13] ), .out0(new_n167));
  and002aa1n02x5               g072(.a(new_n167), .b(new_n160), .o(new_n168));
  160nm_ficinv00aa1n08x5       g073(.clk(new_n168), .clkout(new_n169));
  160nm_ficinv00aa1n08x5       g074(.clk(\b[13] ), .clkout(new_n170));
  oao003aa1n02x5               g075(.a(new_n162), .b(new_n170), .c(new_n164), .carry(new_n171));
  160nm_ficinv00aa1n08x5       g076(.clk(new_n171), .clkout(new_n172));
  aoai13aa1n02x5               g077(.a(new_n172), .b(new_n169), .c(new_n159), .d(new_n148), .o1(new_n173));
  xorb03aa1n02x5               g078(.a(new_n173), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  norp02aa1n02x5               g079(.a(\b[14] ), .b(\a[15] ), .o1(new_n175));
  nanp02aa1n02x5               g080(.a(\b[14] ), .b(\a[15] ), .o1(new_n176));
  norp02aa1n02x5               g081(.a(\b[15] ), .b(\a[16] ), .o1(new_n177));
  nanp02aa1n02x5               g082(.a(\b[15] ), .b(\a[16] ), .o1(new_n178));
  nanb02aa1n02x5               g083(.a(new_n177), .b(new_n178), .out0(new_n179));
  aoai13aa1n02x5               g084(.a(new_n179), .b(new_n175), .c(new_n173), .d(new_n176), .o1(new_n180));
  aoi112aa1n02x5               g085(.a(new_n179), .b(new_n175), .c(new_n173), .d(new_n176), .o1(new_n181));
  nanb02aa1n02x5               g086(.a(new_n181), .b(new_n180), .out0(\s[16] ));
  nano23aa1n02x4               g087(.a(new_n175), .b(new_n177), .c(new_n178), .d(new_n176), .out0(new_n183));
  nanp03aa1n02x5               g088(.a(new_n183), .b(new_n160), .c(new_n167), .o1(new_n184));
  nano32aa1n02x4               g089(.a(new_n184), .b(new_n157), .c(new_n156), .d(new_n99), .out0(new_n185));
  aoai13aa1n02x5               g090(.a(new_n185), .b(new_n155), .c(new_n151), .d(new_n154), .o1(new_n186));
  160nm_fiao0012aa1n02p5x5     g091(.a(new_n177), .b(new_n175), .c(new_n178), .o(new_n187));
  aoi012aa1n02x5               g092(.a(new_n187), .b(new_n183), .c(new_n171), .o1(new_n188));
  aoai13aa1n02x5               g093(.a(new_n188), .b(new_n184), .c(new_n146), .d(new_n147), .o1(new_n189));
  160nm_ficinv00aa1n08x5       g094(.clk(new_n189), .clkout(new_n190));
  nanp02aa1n02x5               g095(.a(new_n186), .b(new_n190), .o1(new_n191));
  xorb03aa1n02x5               g096(.a(new_n191), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  160nm_ficinv00aa1n08x5       g097(.clk(\a[18] ), .clkout(new_n193));
  160nm_ficinv00aa1n08x5       g098(.clk(\a[17] ), .clkout(new_n194));
  160nm_ficinv00aa1n08x5       g099(.clk(\b[16] ), .clkout(new_n195));
  oaoi03aa1n02x5               g100(.a(new_n194), .b(new_n195), .c(new_n191), .o1(new_n196));
  xorb03aa1n02x5               g101(.a(new_n196), .b(\b[17] ), .c(new_n193), .out0(\s[18] ));
  xroi22aa1d04x5               g102(.a(new_n194), .b(\b[16] ), .c(new_n193), .d(\b[17] ), .out0(new_n198));
  160nm_ficinv00aa1n08x5       g103(.clk(new_n198), .clkout(new_n199));
  norp02aa1n02x5               g104(.a(\b[17] ), .b(\a[18] ), .o1(new_n200));
  aoi112aa1n02x5               g105(.a(\b[16] ), .b(\a[17] ), .c(\a[18] ), .d(\b[17] ), .o1(new_n201));
  norp02aa1n02x5               g106(.a(new_n201), .b(new_n200), .o1(new_n202));
  aoai13aa1n02x5               g107(.a(new_n202), .b(new_n199), .c(new_n186), .d(new_n190), .o1(new_n203));
  xorb03aa1n02x5               g108(.a(new_n203), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g109(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  norp02aa1n02x5               g110(.a(\b[18] ), .b(\a[19] ), .o1(new_n206));
  nanp02aa1n02x5               g111(.a(\b[18] ), .b(\a[19] ), .o1(new_n207));
  norp02aa1n02x5               g112(.a(\b[19] ), .b(\a[20] ), .o1(new_n208));
  nanp02aa1n02x5               g113(.a(\b[19] ), .b(\a[20] ), .o1(new_n209));
  norb02aa1n02x5               g114(.a(new_n209), .b(new_n208), .out0(new_n210));
  160nm_ficinv00aa1n08x5       g115(.clk(new_n210), .clkout(new_n211));
  aoai13aa1n02x5               g116(.a(new_n211), .b(new_n206), .c(new_n203), .d(new_n207), .o1(new_n212));
  norb02aa1n02x5               g117(.a(new_n207), .b(new_n206), .out0(new_n213));
  nanp02aa1n02x5               g118(.a(new_n203), .b(new_n213), .o1(new_n214));
  nona22aa1n02x4               g119(.a(new_n214), .b(new_n211), .c(new_n206), .out0(new_n215));
  nanp02aa1n02x5               g120(.a(new_n215), .b(new_n212), .o1(\s[20] ));
  nona23aa1n02x4               g121(.a(new_n209), .b(new_n207), .c(new_n206), .d(new_n208), .out0(new_n217));
  aoi012aa1n02x5               g122(.a(new_n208), .b(new_n206), .c(new_n209), .o1(new_n218));
  oai012aa1n02x5               g123(.a(new_n218), .b(new_n217), .c(new_n202), .o1(new_n219));
  160nm_ficinv00aa1n08x5       g124(.clk(new_n219), .clkout(new_n220));
  nanb02aa1n02x5               g125(.a(new_n217), .b(new_n198), .out0(new_n221));
  aoai13aa1n02x5               g126(.a(new_n220), .b(new_n221), .c(new_n186), .d(new_n190), .o1(new_n222));
  xorb03aa1n02x5               g127(.a(new_n222), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  norp02aa1n02x5               g128(.a(\b[20] ), .b(\a[21] ), .o1(new_n224));
  xorc02aa1n02x5               g129(.a(\a[21] ), .b(\b[20] ), .out0(new_n225));
  xnrc02aa1n02x5               g130(.a(\b[21] ), .b(\a[22] ), .out0(new_n226));
  aoai13aa1n02x5               g131(.a(new_n226), .b(new_n224), .c(new_n222), .d(new_n225), .o1(new_n227));
  nanp02aa1n02x5               g132(.a(new_n222), .b(new_n225), .o1(new_n228));
  nona22aa1n02x4               g133(.a(new_n228), .b(new_n226), .c(new_n224), .out0(new_n229));
  nanp02aa1n02x5               g134(.a(new_n229), .b(new_n227), .o1(\s[22] ));
  nona23aa1n02x4               g135(.a(new_n198), .b(new_n225), .c(new_n226), .d(new_n217), .out0(new_n231));
  oai112aa1n02x5               g136(.a(new_n213), .b(new_n210), .c(new_n201), .d(new_n200), .o1(new_n232));
  nanb02aa1n02x5               g137(.a(new_n226), .b(new_n225), .out0(new_n233));
  160nm_ficinv00aa1n08x5       g138(.clk(\a[22] ), .clkout(new_n234));
  160nm_ficinv00aa1n08x5       g139(.clk(\b[21] ), .clkout(new_n235));
  oaoi03aa1n02x5               g140(.a(new_n234), .b(new_n235), .c(new_n224), .o1(new_n236));
  aoai13aa1n02x5               g141(.a(new_n236), .b(new_n233), .c(new_n232), .d(new_n218), .o1(new_n237));
  160nm_ficinv00aa1n08x5       g142(.clk(new_n237), .clkout(new_n238));
  aoai13aa1n02x5               g143(.a(new_n238), .b(new_n231), .c(new_n186), .d(new_n190), .o1(new_n239));
  xorb03aa1n02x5               g144(.a(new_n239), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g145(.a(\b[22] ), .b(\a[23] ), .o1(new_n241));
  xorc02aa1n02x5               g146(.a(\a[23] ), .b(\b[22] ), .out0(new_n242));
  xorc02aa1n02x5               g147(.a(\a[24] ), .b(\b[23] ), .out0(new_n243));
  160nm_ficinv00aa1n08x5       g148(.clk(new_n243), .clkout(new_n244));
  aoai13aa1n02x5               g149(.a(new_n244), .b(new_n241), .c(new_n239), .d(new_n242), .o1(new_n245));
  nanp02aa1n02x5               g150(.a(new_n239), .b(new_n242), .o1(new_n246));
  nona22aa1n02x4               g151(.a(new_n246), .b(new_n244), .c(new_n241), .out0(new_n247));
  nanp02aa1n02x5               g152(.a(new_n247), .b(new_n245), .o1(\s[24] ));
  norb02aa1n02x5               g153(.a(new_n225), .b(new_n226), .out0(new_n249));
  and002aa1n02x5               g154(.a(new_n243), .b(new_n242), .o(new_n250));
  nano22aa1n02x4               g155(.a(new_n221), .b(new_n250), .c(new_n249), .out0(new_n251));
  aoai13aa1n02x5               g156(.a(new_n251), .b(new_n189), .c(new_n128), .d(new_n185), .o1(new_n252));
  160nm_ficinv00aa1n08x5       g157(.clk(new_n236), .clkout(new_n253));
  aoai13aa1n02x5               g158(.a(new_n250), .b(new_n253), .c(new_n219), .d(new_n249), .o1(new_n254));
  160nm_ficinv00aa1n08x5       g159(.clk(\a[24] ), .clkout(new_n255));
  160nm_ficinv00aa1n08x5       g160(.clk(\b[23] ), .clkout(new_n256));
  oaoi03aa1n02x5               g161(.a(new_n255), .b(new_n256), .c(new_n241), .o1(new_n257));
  nanp02aa1n02x5               g162(.a(new_n254), .b(new_n257), .o1(new_n258));
  nanb02aa1n02x5               g163(.a(new_n258), .b(new_n252), .out0(new_n259));
  xorb03aa1n02x5               g164(.a(new_n259), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g165(.a(\b[24] ), .b(\a[25] ), .o1(new_n261));
  xorc02aa1n02x5               g166(.a(\a[25] ), .b(\b[24] ), .out0(new_n262));
  xorc02aa1n02x5               g167(.a(\a[26] ), .b(\b[25] ), .out0(new_n263));
  160nm_ficinv00aa1n08x5       g168(.clk(new_n263), .clkout(new_n264));
  aoai13aa1n02x5               g169(.a(new_n264), .b(new_n261), .c(new_n259), .d(new_n262), .o1(new_n265));
  aoai13aa1n02x5               g170(.a(new_n262), .b(new_n258), .c(new_n191), .d(new_n251), .o1(new_n266));
  nona22aa1n02x4               g171(.a(new_n266), .b(new_n264), .c(new_n261), .out0(new_n267));
  nanp02aa1n02x5               g172(.a(new_n265), .b(new_n267), .o1(\s[26] ));
  and002aa1n02x5               g173(.a(new_n263), .b(new_n262), .o(new_n269));
  nano22aa1n02x4               g174(.a(new_n231), .b(new_n250), .c(new_n269), .out0(new_n270));
  aoai13aa1n02x5               g175(.a(new_n270), .b(new_n189), .c(new_n128), .d(new_n185), .o1(new_n271));
  160nm_ficinv00aa1n08x5       g176(.clk(new_n257), .clkout(new_n272));
  aoai13aa1n02x5               g177(.a(new_n269), .b(new_n272), .c(new_n237), .d(new_n250), .o1(new_n273));
  aoi112aa1n02x5               g178(.a(\b[24] ), .b(\a[25] ), .c(\a[26] ), .d(\b[25] ), .o1(new_n274));
  oab012aa1n02x4               g179(.a(new_n274), .b(\a[26] ), .c(\b[25] ), .out0(new_n275));
  nanp03aa1n02x5               g180(.a(new_n271), .b(new_n273), .c(new_n275), .o1(new_n276));
  xorb03aa1n02x5               g181(.a(new_n276), .b(\b[26] ), .c(\a[27] ), .out0(\s[27] ));
  norp02aa1n02x5               g182(.a(\b[26] ), .b(\a[27] ), .o1(new_n278));
  xorc02aa1n02x5               g183(.a(\a[27] ), .b(\b[26] ), .out0(new_n279));
  xnrc02aa1n02x5               g184(.a(\b[27] ), .b(\a[28] ), .out0(new_n280));
  aoai13aa1n02x5               g185(.a(new_n280), .b(new_n278), .c(new_n276), .d(new_n279), .o1(new_n281));
  aobi12aa1n02x5               g186(.a(new_n270), .b(new_n186), .c(new_n190), .out0(new_n282));
  160nm_ficinv00aa1n08x5       g187(.clk(new_n269), .clkout(new_n283));
  aoai13aa1n02x5               g188(.a(new_n275), .b(new_n283), .c(new_n254), .d(new_n257), .o1(new_n284));
  oai012aa1n02x5               g189(.a(new_n279), .b(new_n284), .c(new_n282), .o1(new_n285));
  nona22aa1n02x4               g190(.a(new_n285), .b(new_n280), .c(new_n278), .out0(new_n286));
  nanp02aa1n02x5               g191(.a(new_n281), .b(new_n286), .o1(\s[28] ));
  norb02aa1n02x5               g192(.a(new_n279), .b(new_n280), .out0(new_n288));
  oai012aa1n02x5               g193(.a(new_n288), .b(new_n284), .c(new_n282), .o1(new_n289));
  aob012aa1n02x5               g194(.a(new_n278), .b(\b[27] ), .c(\a[28] ), .out0(new_n290));
  oa0012aa1n02x5               g195(.a(new_n290), .b(\b[27] ), .c(\a[28] ), .o(new_n291));
  160nm_ficinv00aa1n08x5       g196(.clk(new_n291), .clkout(new_n292));
  xnrc02aa1n02x5               g197(.a(\b[28] ), .b(\a[29] ), .out0(new_n293));
  nona22aa1n02x4               g198(.a(new_n289), .b(new_n292), .c(new_n293), .out0(new_n294));
  aoai13aa1n02x5               g199(.a(new_n293), .b(new_n292), .c(new_n276), .d(new_n288), .o1(new_n295));
  nanp02aa1n02x5               g200(.a(new_n295), .b(new_n294), .o1(\s[29] ));
  xorb03aa1n02x5               g201(.a(new_n108), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g202(.a(new_n279), .b(new_n293), .c(new_n280), .out0(new_n298));
  oaoi03aa1n02x5               g203(.a(\a[29] ), .b(\b[28] ), .c(new_n291), .o1(new_n299));
  xnrc02aa1n02x5               g204(.a(\b[29] ), .b(\a[30] ), .out0(new_n300));
  aoai13aa1n02x5               g205(.a(new_n300), .b(new_n299), .c(new_n276), .d(new_n298), .o1(new_n301));
  oai012aa1n02x5               g206(.a(new_n298), .b(new_n284), .c(new_n282), .o1(new_n302));
  nona22aa1n02x4               g207(.a(new_n302), .b(new_n299), .c(new_n300), .out0(new_n303));
  nanp02aa1n02x5               g208(.a(new_n301), .b(new_n303), .o1(\s[30] ));
  norb02aa1n02x5               g209(.a(new_n298), .b(new_n300), .out0(new_n305));
  oai012aa1n02x5               g210(.a(new_n305), .b(new_n284), .c(new_n282), .o1(new_n306));
  nanb02aa1n02x5               g211(.a(new_n300), .b(new_n299), .out0(new_n307));
  oai012aa1n02x5               g212(.a(new_n307), .b(\b[29] ), .c(\a[30] ), .o1(new_n308));
  xnrc02aa1n02x5               g213(.a(\b[30] ), .b(\a[31] ), .out0(new_n309));
  nona22aa1n02x4               g214(.a(new_n306), .b(new_n308), .c(new_n309), .out0(new_n310));
  aoai13aa1n02x5               g215(.a(new_n309), .b(new_n308), .c(new_n276), .d(new_n305), .o1(new_n311));
  nanp02aa1n02x5               g216(.a(new_n311), .b(new_n310), .o1(\s[31] ));
  xorb03aa1n02x5               g217(.a(new_n110), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  aoi012aa1n02x5               g218(.a(new_n104), .b(new_n110), .c(new_n105), .o1(new_n314));
  xnrb03aa1n02x5               g219(.a(new_n314), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g220(.a(new_n151), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  aoi012aa1n02x5               g221(.a(new_n120), .b(new_n151), .c(new_n121), .o1(new_n317));
  xnrb03aa1n02x5               g222(.a(new_n317), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  aoai13aa1n02x5               g223(.a(new_n124), .b(new_n153), .c(new_n111), .d(new_n112), .o1(new_n319));
  xorb03aa1n02x5               g224(.a(new_n319), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoi012aa1n02x5               g225(.a(new_n115), .b(new_n319), .c(new_n116), .o1(new_n321));
  xnrb03aa1n02x5               g226(.a(new_n321), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xorb03aa1n02x5               g227(.a(new_n128), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule

