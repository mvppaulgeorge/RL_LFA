// Benchmark "adder" written by ABC on Thu Jul 11 11:13:54 2024

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
    new_n133, new_n134, new_n135, new_n137, new_n139, new_n140, new_n141,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n150, new_n151, new_n152, new_n153, new_n154, new_n156, new_n157,
    new_n158, new_n159, new_n160, new_n161, new_n162, new_n163, new_n164,
    new_n165, new_n167, new_n168, new_n169, new_n170, new_n172, new_n173,
    new_n174, new_n175, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n183, new_n184, new_n185, new_n186, new_n188, new_n189,
    new_n190, new_n191, new_n192, new_n193, new_n194, new_n195, new_n196,
    new_n199, new_n200, new_n201, new_n202, new_n203, new_n204, new_n206,
    new_n207, new_n208, new_n209, new_n210, new_n211, new_n212, new_n213,
    new_n214, new_n215, new_n217, new_n218, new_n219, new_n220, new_n221,
    new_n223, new_n224, new_n225, new_n226, new_n227, new_n228, new_n229,
    new_n230, new_n232, new_n233, new_n234, new_n235, new_n236, new_n238,
    new_n239, new_n240, new_n241, new_n242, new_n243, new_n244, new_n245,
    new_n246, new_n247, new_n248, new_n249, new_n250, new_n252, new_n253,
    new_n254, new_n255, new_n256, new_n258, new_n259, new_n260, new_n261,
    new_n262, new_n263, new_n264, new_n265, new_n267, new_n268, new_n269,
    new_n270, new_n271, new_n272, new_n273, new_n274, new_n275, new_n276,
    new_n277, new_n278, new_n280, new_n281, new_n282, new_n283, new_n284,
    new_n285, new_n286, new_n287, new_n290, new_n291, new_n292, new_n293,
    new_n294, new_n295, new_n296, new_n297, new_n299, new_n300, new_n301,
    new_n302, new_n303, new_n304, new_n305, new_n306, new_n309, new_n312,
    new_n313, new_n315, new_n317;
  160nm_ficinv00aa1n08x5       g000(.clk(\a[0] ), .clkout(\s[0] ));
  norp02aa1n02x5               g001(.a(\b[9] ), .b(\a[10] ), .o1(new_n97));
  and002aa1n02x5               g002(.a(\b[9] ), .b(\a[10] ), .o(new_n98));
  norp02aa1n02x5               g003(.a(new_n98), .b(new_n97), .o1(new_n99));
  nanp02aa1n02x5               g004(.a(\b[8] ), .b(\a[9] ), .o1(new_n100));
  norp02aa1n02x5               g005(.a(\b[8] ), .b(\a[9] ), .o1(new_n101));
  and002aa1n02x5               g006(.a(\b[3] ), .b(\a[4] ), .o(new_n102));
  160nm_ficinv00aa1n08x5       g007(.clk(\a[3] ), .clkout(new_n103));
  160nm_ficinv00aa1n08x5       g008(.clk(\b[2] ), .clkout(new_n104));
  nanp02aa1n02x5               g009(.a(new_n104), .b(new_n103), .o1(new_n105));
  nanp02aa1n02x5               g010(.a(\b[2] ), .b(\a[3] ), .o1(new_n106));
  nanp02aa1n02x5               g011(.a(new_n105), .b(new_n106), .o1(new_n107));
  nanp02aa1n02x5               g012(.a(\b[1] ), .b(\a[2] ), .o1(new_n108));
  norp02aa1n02x5               g013(.a(\b[1] ), .b(\a[2] ), .o1(new_n109));
  nanp02aa1n02x5               g014(.a(\b[0] ), .b(\a[1] ), .o1(new_n110));
  oai012aa1n02x5               g015(.a(new_n108), .b(new_n109), .c(new_n110), .o1(new_n111));
  160nm_ficinv00aa1n08x5       g016(.clk(\b[3] ), .clkout(new_n112));
  aboi22aa1n03x5               g017(.a(\a[4] ), .b(new_n112), .c(new_n103), .d(new_n104), .out0(new_n113));
  oai012aa1n02x5               g018(.a(new_n113), .b(new_n111), .c(new_n107), .o1(new_n114));
  norp02aa1n02x5               g019(.a(\b[7] ), .b(\a[8] ), .o1(new_n115));
  nanp02aa1n02x5               g020(.a(\b[7] ), .b(\a[8] ), .o1(new_n116));
  norp02aa1n02x5               g021(.a(\b[6] ), .b(\a[7] ), .o1(new_n117));
  nanp02aa1n02x5               g022(.a(\b[6] ), .b(\a[7] ), .o1(new_n118));
  nona23aa1n02x4               g023(.a(new_n118), .b(new_n116), .c(new_n115), .d(new_n117), .out0(new_n119));
  xnrc02aa1n02x5               g024(.a(\b[5] ), .b(\a[6] ), .out0(new_n120));
  xnrc02aa1n02x5               g025(.a(\b[4] ), .b(\a[5] ), .out0(new_n121));
  norp02aa1n02x5               g026(.a(new_n121), .b(new_n120), .o1(new_n122));
  nona23aa1n02x4               g027(.a(new_n114), .b(new_n122), .c(new_n119), .d(new_n102), .out0(new_n123));
  norp02aa1n02x5               g028(.a(\b[5] ), .b(\a[6] ), .o1(new_n124));
  aoi112aa1n02x5               g029(.a(\b[4] ), .b(\a[5] ), .c(\a[6] ), .d(\b[5] ), .o1(new_n125));
  norp02aa1n02x5               g030(.a(new_n125), .b(new_n124), .o1(new_n126));
  160nm_fiao0012aa1n02p5x5     g031(.a(new_n115), .b(new_n117), .c(new_n116), .o(new_n127));
  oabi12aa1n02x5               g032(.a(new_n127), .b(new_n119), .c(new_n126), .out0(new_n128));
  nona22aa1n02x4               g033(.a(new_n123), .b(new_n128), .c(new_n101), .out0(new_n129));
  xobna2aa1n03x5               g034(.a(new_n99), .b(new_n129), .c(new_n100), .out0(\s[10] ));
  norp02aa1n02x5               g035(.a(\b[10] ), .b(\a[11] ), .o1(new_n131));
  nanp02aa1n02x5               g036(.a(\b[10] ), .b(\a[11] ), .o1(new_n132));
  norb02aa1n02x5               g037(.a(new_n132), .b(new_n131), .out0(new_n133));
  160nm_ficinv00aa1n08x5       g038(.clk(new_n98), .clkout(new_n134));
  aoai13aa1n02x5               g039(.a(new_n134), .b(new_n97), .c(new_n129), .d(new_n100), .o1(new_n135));
  xnrc02aa1n02x5               g040(.a(new_n135), .b(new_n133), .out0(\s[11] ));
  oaoi03aa1n02x5               g041(.a(\a[11] ), .b(\b[10] ), .c(new_n135), .o1(new_n137));
  xorb03aa1n02x5               g042(.a(new_n137), .b(\b[11] ), .c(\a[12] ), .out0(\s[12] ));
  160nm_ficinv00aa1n08x5       g043(.clk(new_n128), .clkout(new_n139));
  norp02aa1n02x5               g044(.a(\b[11] ), .b(\a[12] ), .o1(new_n140));
  nanp02aa1n02x5               g045(.a(\b[11] ), .b(\a[12] ), .o1(new_n141));
  nano23aa1n02x4               g046(.a(new_n131), .b(new_n140), .c(new_n141), .d(new_n132), .out0(new_n142));
  norb02aa1n02x5               g047(.a(new_n100), .b(new_n101), .out0(new_n143));
  nanp03aa1n02x5               g048(.a(new_n142), .b(new_n99), .c(new_n143), .o1(new_n144));
  oab012aa1n02x4               g049(.a(new_n98), .b(new_n97), .c(new_n101), .out0(new_n145));
  aoi012aa1n02x5               g050(.a(new_n140), .b(new_n131), .c(new_n141), .o1(new_n146));
  aobi12aa1n02x5               g051(.a(new_n146), .b(new_n142), .c(new_n145), .out0(new_n147));
  aoai13aa1n02x5               g052(.a(new_n147), .b(new_n144), .c(new_n123), .d(new_n139), .o1(new_n148));
  xorb03aa1n02x5               g053(.a(new_n148), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  160nm_ficinv00aa1n08x5       g054(.clk(\a[14] ), .clkout(new_n150));
  norp02aa1n02x5               g055(.a(\b[12] ), .b(\a[13] ), .o1(new_n151));
  xnrc02aa1n02x5               g056(.a(\b[12] ), .b(\a[13] ), .out0(new_n152));
  160nm_ficinv00aa1n08x5       g057(.clk(new_n152), .clkout(new_n153));
  aoi012aa1n02x5               g058(.a(new_n151), .b(new_n148), .c(new_n153), .o1(new_n154));
  xorb03aa1n02x5               g059(.a(new_n154), .b(\b[13] ), .c(new_n150), .out0(\s[14] ));
  norp02aa1n02x5               g060(.a(\b[14] ), .b(\a[15] ), .o1(new_n156));
  nanp02aa1n02x5               g061(.a(\b[14] ), .b(\a[15] ), .o1(new_n157));
  norb02aa1n02x5               g062(.a(new_n157), .b(new_n156), .out0(new_n158));
  xnrc02aa1n02x5               g063(.a(\b[13] ), .b(\a[14] ), .out0(new_n159));
  norp02aa1n02x5               g064(.a(new_n159), .b(new_n152), .o1(new_n160));
  160nm_ficinv00aa1n08x5       g065(.clk(\b[13] ), .clkout(new_n161));
  oaoi03aa1n02x5               g066(.a(new_n150), .b(new_n161), .c(new_n151), .o1(new_n162));
  160nm_ficinv00aa1n08x5       g067(.clk(new_n162), .clkout(new_n163));
  aoai13aa1n02x5               g068(.a(new_n158), .b(new_n163), .c(new_n148), .d(new_n160), .o1(new_n164));
  aoi112aa1n02x5               g069(.a(new_n158), .b(new_n163), .c(new_n148), .d(new_n160), .o1(new_n165));
  norb02aa1n02x5               g070(.a(new_n164), .b(new_n165), .out0(\s[15] ));
  160nm_ficinv00aa1n08x5       g071(.clk(new_n156), .clkout(new_n167));
  norp02aa1n02x5               g072(.a(\b[15] ), .b(\a[16] ), .o1(new_n168));
  nanp02aa1n02x5               g073(.a(\b[15] ), .b(\a[16] ), .o1(new_n169));
  norb02aa1n02x5               g074(.a(new_n169), .b(new_n168), .out0(new_n170));
  xnbna2aa1n03x5               g075(.a(new_n170), .b(new_n164), .c(new_n167), .out0(\s[16] ));
  oaoi13aa1n02x5               g076(.a(new_n102), .b(new_n113), .c(new_n111), .d(new_n107), .o1(new_n172));
  norp03aa1n02x5               g077(.a(new_n119), .b(new_n120), .c(new_n121), .o1(new_n173));
  nano23aa1n02x4               g078(.a(new_n156), .b(new_n168), .c(new_n169), .d(new_n157), .out0(new_n174));
  nona22aa1n02x4               g079(.a(new_n174), .b(new_n159), .c(new_n152), .out0(new_n175));
  norp02aa1n02x5               g080(.a(new_n175), .b(new_n144), .o1(new_n176));
  aoai13aa1n02x5               g081(.a(new_n176), .b(new_n128), .c(new_n172), .d(new_n173), .o1(new_n177));
  aoi012aa1n02x5               g082(.a(new_n168), .b(new_n156), .c(new_n169), .o1(new_n178));
  oaib12aa1n02x5               g083(.a(new_n178), .b(new_n162), .c(new_n174), .out0(new_n179));
  oab012aa1n02x4               g084(.a(new_n179), .b(new_n147), .c(new_n175), .out0(new_n180));
  nanp02aa1n02x5               g085(.a(new_n177), .b(new_n180), .o1(new_n181));
  xorb03aa1n02x5               g086(.a(new_n181), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  160nm_ficinv00aa1n08x5       g087(.clk(\a[18] ), .clkout(new_n183));
  160nm_ficinv00aa1n08x5       g088(.clk(\a[17] ), .clkout(new_n184));
  160nm_ficinv00aa1n08x5       g089(.clk(\b[16] ), .clkout(new_n185));
  oaoi03aa1n02x5               g090(.a(new_n184), .b(new_n185), .c(new_n181), .o1(new_n186));
  xorb03aa1n02x5               g091(.a(new_n186), .b(\b[17] ), .c(new_n183), .out0(\s[18] ));
  xroi22aa1d04x5               g092(.a(new_n184), .b(\b[16] ), .c(new_n183), .d(\b[17] ), .out0(new_n188));
  nanp02aa1n02x5               g093(.a(\b[17] ), .b(\a[18] ), .o1(new_n189));
  nona22aa1n02x4               g094(.a(new_n189), .b(\b[16] ), .c(\a[17] ), .out0(new_n190));
  oaib12aa1n02x5               g095(.a(new_n190), .b(\b[17] ), .c(new_n183), .out0(new_n191));
  norp02aa1n02x5               g096(.a(\b[18] ), .b(\a[19] ), .o1(new_n192));
  nanp02aa1n02x5               g097(.a(\b[18] ), .b(\a[19] ), .o1(new_n193));
  norb02aa1n02x5               g098(.a(new_n193), .b(new_n192), .out0(new_n194));
  aoai13aa1n02x5               g099(.a(new_n194), .b(new_n191), .c(new_n181), .d(new_n188), .o1(new_n195));
  aoi112aa1n02x5               g100(.a(new_n194), .b(new_n191), .c(new_n181), .d(new_n188), .o1(new_n196));
  norb02aa1n02x5               g101(.a(new_n195), .b(new_n196), .out0(\s[19] ));
  xnrc02aa1n02x5               g102(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  norp02aa1n02x5               g103(.a(\b[19] ), .b(\a[20] ), .o1(new_n199));
  nanp02aa1n02x5               g104(.a(\b[19] ), .b(\a[20] ), .o1(new_n200));
  norb02aa1n02x5               g105(.a(new_n200), .b(new_n199), .out0(new_n201));
  nona22aa1n02x4               g106(.a(new_n195), .b(new_n201), .c(new_n192), .out0(new_n202));
  orn002aa1n02x5               g107(.a(\a[19] ), .b(\b[18] ), .o(new_n203));
  aobi12aa1n02x5               g108(.a(new_n201), .b(new_n195), .c(new_n203), .out0(new_n204));
  norb02aa1n02x5               g109(.a(new_n202), .b(new_n204), .out0(\s[20] ));
  nano23aa1n02x4               g110(.a(new_n192), .b(new_n199), .c(new_n200), .d(new_n193), .out0(new_n206));
  nanp02aa1n02x5               g111(.a(new_n188), .b(new_n206), .o1(new_n207));
  norp02aa1n02x5               g112(.a(\b[17] ), .b(\a[18] ), .o1(new_n208));
  aoi013aa1n02x4               g113(.a(new_n208), .b(new_n189), .c(new_n184), .d(new_n185), .o1(new_n209));
  nona23aa1n02x4               g114(.a(new_n200), .b(new_n193), .c(new_n192), .d(new_n199), .out0(new_n210));
  oaoi03aa1n02x5               g115(.a(\a[20] ), .b(\b[19] ), .c(new_n203), .o1(new_n211));
  160nm_ficinv00aa1n08x5       g116(.clk(new_n211), .clkout(new_n212));
  oai012aa1n02x5               g117(.a(new_n212), .b(new_n210), .c(new_n209), .o1(new_n213));
  160nm_ficinv00aa1n08x5       g118(.clk(new_n213), .clkout(new_n214));
  aoai13aa1n02x5               g119(.a(new_n214), .b(new_n207), .c(new_n177), .d(new_n180), .o1(new_n215));
  xorb03aa1n02x5               g120(.a(new_n215), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  norp02aa1n02x5               g121(.a(\b[20] ), .b(\a[21] ), .o1(new_n217));
  xorc02aa1n02x5               g122(.a(\a[21] ), .b(\b[20] ), .out0(new_n218));
  xorc02aa1n02x5               g123(.a(\a[22] ), .b(\b[21] ), .out0(new_n219));
  aoi112aa1n02x5               g124(.a(new_n217), .b(new_n219), .c(new_n215), .d(new_n218), .o1(new_n220));
  aoai13aa1n02x5               g125(.a(new_n219), .b(new_n217), .c(new_n215), .d(new_n218), .o1(new_n221));
  norb02aa1n02x5               g126(.a(new_n221), .b(new_n220), .out0(\s[22] ));
  160nm_ficinv00aa1n08x5       g127(.clk(\a[21] ), .clkout(new_n223));
  160nm_ficinv00aa1n08x5       g128(.clk(\a[22] ), .clkout(new_n224));
  xroi22aa1d04x5               g129(.a(new_n223), .b(\b[20] ), .c(new_n224), .d(\b[21] ), .out0(new_n225));
  nanp03aa1n02x5               g130(.a(new_n225), .b(new_n188), .c(new_n206), .o1(new_n226));
  160nm_ficinv00aa1n08x5       g131(.clk(\b[21] ), .clkout(new_n227));
  oao003aa1n02x5               g132(.a(new_n224), .b(new_n227), .c(new_n217), .carry(new_n228));
  aoi012aa1n02x5               g133(.a(new_n228), .b(new_n213), .c(new_n225), .o1(new_n229));
  aoai13aa1n02x5               g134(.a(new_n229), .b(new_n226), .c(new_n177), .d(new_n180), .o1(new_n230));
  xorb03aa1n02x5               g135(.a(new_n230), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g136(.a(\b[22] ), .b(\a[23] ), .o1(new_n232));
  xorc02aa1n02x5               g137(.a(\a[23] ), .b(\b[22] ), .out0(new_n233));
  xorc02aa1n02x5               g138(.a(\a[24] ), .b(\b[23] ), .out0(new_n234));
  aoi112aa1n02x5               g139(.a(new_n232), .b(new_n234), .c(new_n230), .d(new_n233), .o1(new_n235));
  aoai13aa1n02x5               g140(.a(new_n234), .b(new_n232), .c(new_n230), .d(new_n233), .o1(new_n236));
  norb02aa1n02x5               g141(.a(new_n236), .b(new_n235), .out0(\s[24] ));
  160nm_ficinv00aa1n08x5       g142(.clk(\a[23] ), .clkout(new_n238));
  160nm_ficinv00aa1n08x5       g143(.clk(\a[24] ), .clkout(new_n239));
  xroi22aa1d04x5               g144(.a(new_n238), .b(\b[22] ), .c(new_n239), .d(\b[23] ), .out0(new_n240));
  nano22aa1n02x4               g145(.a(new_n207), .b(new_n225), .c(new_n240), .out0(new_n241));
  aoai13aa1n02x5               g146(.a(new_n225), .b(new_n211), .c(new_n206), .d(new_n191), .o1(new_n242));
  160nm_ficinv00aa1n08x5       g147(.clk(new_n228), .clkout(new_n243));
  160nm_ficinv00aa1n08x5       g148(.clk(new_n240), .clkout(new_n244));
  oai022aa1n02x5               g149(.a(\a[23] ), .b(\b[22] ), .c(\b[23] ), .d(\a[24] ), .o1(new_n245));
  oaib12aa1n02x5               g150(.a(new_n245), .b(new_n239), .c(\b[23] ), .out0(new_n246));
  aoai13aa1n02x5               g151(.a(new_n246), .b(new_n244), .c(new_n242), .d(new_n243), .o1(new_n247));
  xorc02aa1n02x5               g152(.a(\a[25] ), .b(\b[24] ), .out0(new_n248));
  aoai13aa1n02x5               g153(.a(new_n248), .b(new_n247), .c(new_n181), .d(new_n241), .o1(new_n249));
  aoi112aa1n02x5               g154(.a(new_n248), .b(new_n247), .c(new_n181), .d(new_n241), .o1(new_n250));
  norb02aa1n02x5               g155(.a(new_n249), .b(new_n250), .out0(\s[25] ));
  norp02aa1n02x5               g156(.a(\b[24] ), .b(\a[25] ), .o1(new_n252));
  xorc02aa1n02x5               g157(.a(\a[26] ), .b(\b[25] ), .out0(new_n253));
  nona22aa1n02x4               g158(.a(new_n249), .b(new_n253), .c(new_n252), .out0(new_n254));
  160nm_ficinv00aa1n08x5       g159(.clk(new_n252), .clkout(new_n255));
  aobi12aa1n02x5               g160(.a(new_n253), .b(new_n249), .c(new_n255), .out0(new_n256));
  norb02aa1n02x5               g161(.a(new_n254), .b(new_n256), .out0(\s[26] ));
  160nm_ficinv00aa1n08x5       g162(.clk(\a[25] ), .clkout(new_n258));
  160nm_ficinv00aa1n08x5       g163(.clk(\a[26] ), .clkout(new_n259));
  xroi22aa1d04x5               g164(.a(new_n258), .b(\b[24] ), .c(new_n259), .d(\b[25] ), .out0(new_n260));
  nano32aa1n02x4               g165(.a(new_n207), .b(new_n260), .c(new_n225), .d(new_n240), .out0(new_n261));
  nanp02aa1n02x5               g166(.a(new_n181), .b(new_n261), .o1(new_n262));
  oao003aa1n02x5               g167(.a(\a[26] ), .b(\b[25] ), .c(new_n255), .carry(new_n263));
  aobi12aa1n02x5               g168(.a(new_n263), .b(new_n247), .c(new_n260), .out0(new_n264));
  xorc02aa1n02x5               g169(.a(\a[27] ), .b(\b[26] ), .out0(new_n265));
  xnbna2aa1n03x5               g170(.a(new_n265), .b(new_n264), .c(new_n262), .out0(\s[27] ));
  norp02aa1n02x5               g171(.a(\b[26] ), .b(\a[27] ), .o1(new_n267));
  160nm_ficinv00aa1n08x5       g172(.clk(new_n267), .clkout(new_n268));
  160nm_ficinv00aa1n08x5       g173(.clk(new_n265), .clkout(new_n269));
  aoi012aa1n02x5               g174(.a(new_n269), .b(new_n264), .c(new_n262), .o1(new_n270));
  xnrc02aa1n02x5               g175(.a(\b[27] ), .b(\a[28] ), .out0(new_n271));
  nano22aa1n02x4               g176(.a(new_n270), .b(new_n268), .c(new_n271), .out0(new_n272));
  aobi12aa1n02x5               g177(.a(new_n261), .b(new_n177), .c(new_n180), .out0(new_n273));
  aoai13aa1n02x5               g178(.a(new_n240), .b(new_n228), .c(new_n213), .d(new_n225), .o1(new_n274));
  160nm_ficinv00aa1n08x5       g179(.clk(new_n260), .clkout(new_n275));
  aoai13aa1n02x5               g180(.a(new_n263), .b(new_n275), .c(new_n274), .d(new_n246), .o1(new_n276));
  oai012aa1n02x5               g181(.a(new_n265), .b(new_n276), .c(new_n273), .o1(new_n277));
  aoi012aa1n02x5               g182(.a(new_n271), .b(new_n277), .c(new_n268), .o1(new_n278));
  norp02aa1n02x5               g183(.a(new_n278), .b(new_n272), .o1(\s[28] ));
  norb02aa1n02x5               g184(.a(new_n265), .b(new_n271), .out0(new_n280));
  oai012aa1n02x5               g185(.a(new_n280), .b(new_n276), .c(new_n273), .o1(new_n281));
  oao003aa1n02x5               g186(.a(\a[28] ), .b(\b[27] ), .c(new_n268), .carry(new_n282));
  xnrc02aa1n02x5               g187(.a(\b[28] ), .b(\a[29] ), .out0(new_n283));
  aoi012aa1n02x5               g188(.a(new_n283), .b(new_n281), .c(new_n282), .o1(new_n284));
  160nm_ficinv00aa1n08x5       g189(.clk(new_n280), .clkout(new_n285));
  aoi012aa1n02x5               g190(.a(new_n285), .b(new_n264), .c(new_n262), .o1(new_n286));
  nano22aa1n02x4               g191(.a(new_n286), .b(new_n282), .c(new_n283), .out0(new_n287));
  norp02aa1n02x5               g192(.a(new_n284), .b(new_n287), .o1(\s[29] ));
  xorb03aa1n02x5               g193(.a(new_n110), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g194(.a(new_n265), .b(new_n283), .c(new_n271), .out0(new_n290));
  oai012aa1n02x5               g195(.a(new_n290), .b(new_n276), .c(new_n273), .o1(new_n291));
  oao003aa1n02x5               g196(.a(\a[29] ), .b(\b[28] ), .c(new_n282), .carry(new_n292));
  xnrc02aa1n02x5               g197(.a(\b[29] ), .b(\a[30] ), .out0(new_n293));
  aoi012aa1n02x5               g198(.a(new_n293), .b(new_n291), .c(new_n292), .o1(new_n294));
  160nm_ficinv00aa1n08x5       g199(.clk(new_n290), .clkout(new_n295));
  aoi012aa1n02x5               g200(.a(new_n295), .b(new_n264), .c(new_n262), .o1(new_n296));
  nano22aa1n02x4               g201(.a(new_n296), .b(new_n292), .c(new_n293), .out0(new_n297));
  norp02aa1n02x5               g202(.a(new_n294), .b(new_n297), .o1(\s[30] ));
  norb02aa1n02x5               g203(.a(new_n290), .b(new_n293), .out0(new_n299));
  160nm_ficinv00aa1n08x5       g204(.clk(new_n299), .clkout(new_n300));
  aoi012aa1n02x5               g205(.a(new_n300), .b(new_n264), .c(new_n262), .o1(new_n301));
  oao003aa1n02x5               g206(.a(\a[30] ), .b(\b[29] ), .c(new_n292), .carry(new_n302));
  xnrc02aa1n02x5               g207(.a(\b[30] ), .b(\a[31] ), .out0(new_n303));
  nano22aa1n02x4               g208(.a(new_n301), .b(new_n302), .c(new_n303), .out0(new_n304));
  oai012aa1n02x5               g209(.a(new_n299), .b(new_n276), .c(new_n273), .o1(new_n305));
  aoi012aa1n02x5               g210(.a(new_n303), .b(new_n305), .c(new_n302), .o1(new_n306));
  norp02aa1n02x5               g211(.a(new_n306), .b(new_n304), .o1(\s[31] ));
  xnbna2aa1n03x5               g212(.a(new_n111), .b(new_n105), .c(new_n106), .out0(\s[3] ));
  oaoi03aa1n02x5               g213(.a(\a[3] ), .b(\b[2] ), .c(new_n111), .o1(new_n309));
  xorb03aa1n02x5               g214(.a(new_n309), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g215(.a(new_n172), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  orn002aa1n02x5               g216(.a(\a[5] ), .b(\b[4] ), .o(new_n312));
  nona22aa1n02x4               g217(.a(new_n114), .b(new_n121), .c(new_n102), .out0(new_n313));
  xobna2aa1n03x5               g218(.a(new_n120), .b(new_n313), .c(new_n312), .out0(\s[6] ));
  aobi12aa1n02x5               g219(.a(new_n126), .b(new_n172), .c(new_n122), .out0(new_n315));
  xnrb03aa1n02x5               g220(.a(new_n315), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  oaoi03aa1n02x5               g221(.a(\a[7] ), .b(\b[6] ), .c(new_n315), .o1(new_n317));
  xorb03aa1n02x5               g222(.a(new_n317), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xnbna2aa1n03x5               g223(.a(new_n143), .b(new_n123), .c(new_n139), .out0(\s[9] ));
endmodule


